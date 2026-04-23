#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "SLAMMath.hpp"

#include "../../src/threadloop.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/data_format.hpp"
#include "../../src/data_format_opencv.hpp"

#include "../../src/stoplight.hpp"

using namespace ILLIXR;
using namespace OpenVINS;

LOG_MODULE_REGISTER(openvins, LOG_LEVEL_INF);

K_THREAD_STACK_DEFINE(openvins_stack, 16777216);

// ── Queue definitions (producers reference these via openvins_queues.hpp) ─────
K_MSGQ_DEFINE(openvins_imu_queue, sizeof(ImuMsg*), 500, 4);
K_MSGQ_DEFINE(openvins_cam_queue, sizeof(CamMsg*), 50, 4);

static constexpr uint32_t kExpectedCamFrames   = 50;
static constexpr size_t   kImuSamplesPerWindow = 10;

// ==============================================================================
// VIO CONFIGURATION (EuRoC calibration)
// ==============================================================================
VIOConfig create_vio_config() {
    VIOConfig config;

    config.cam0.fx = 458.654;  config.cam0.fy = 457.296;
    config.cam0.cx = 367.215;  config.cam0.cy = 248.375;
    config.cam0.k1 = -0.28340811; config.cam0.k2 =  0.07395907;
    config.cam0.p1 =  0.00019359; config.cam0.p2 =  1.76187114e-05;

    config.cam1.fx = 457.587;  config.cam1.fy = 456.134;
    config.cam1.cx = 379.999;  config.cam1.cy = 255.238;
    config.cam1.k1 = -0.28368365; config.cam1.k2 =  0.07451284;
    config.cam1.p1 = -0.00010473; config.cam1.p2 = -3.55590700e-05;

    Eigen::Matrix4d T_C0toI;
    T_C0toI <<  0.0148655429818, -0.999880929698,  0.00414029679422, -0.0216401454975,
                0.999557249008,   0.0149672133247,  0.025715529948,   -0.064676986768,
               -0.0257744366974,  0.00375618835797, 0.999660727178,    0.00981073058949,
                0, 0, 0, 1;
    config.R_ItoC0 = T_C0toI.block<3,3>(0,0).transpose();
    config.p_C0inI = -config.R_ItoC0 * T_C0toI.block<3,1>(0,3);

    Eigen::Matrix4d T_C1toI;
    T_C1toI <<  0.0125552670891, -0.999755099723,  0.0182237714554, -0.0198435579556,
                0.999598781151,   0.0130119051815,  0.0251588363115,  0.0453689425024,
               -0.0253898008918,  0.0179005838253,  0.999517347078,   0.00786212447038,
                0, 0, 0, 1;
    config.R_ItoC1 = T_C1toI.block<3,3>(0,0).transpose();
    config.p_C1inI = -config.R_ItoC1 * T_C1toI.block<3,1>(0,3);

    config.sigma_gyro       = 0.00016968;
    config.sigma_accel      = 0.002;
    config.sigma_gyro_bias  = 1.9393e-05;
    config.sigma_accel_bias = 0.003;
    config.sigma_pixel      = 1.0;
    config.gravity << 0, 0, -9.81;

    config.max_features           = 150;
    config.min_features           = 50;
    config.max_clone_size         = 20;
    config.min_track_length       = 3;
    config.max_reprojection_error = 2.0;
    config.template_size          = 15;
    config.search_radius          = 20;
    config.stereo_search_radius   = 60;
    config.match_threshold        = 0.8;
    config.fb_check_thresh        = 2.0;

    return config;
}

// ==============================================================================
// OPENVINS PLUGIN
//
// No pub/sub subscriptions.  Producers push ImuMsg*/CamMsg* directly
// into openvins_imu_queue / openvins_cam_queue.  This plugin pulls
// from the queues in lock-step:
//
//   1. give stoplight_imu  → IMU thread sends 10 samples into queue
//   2. blocking-read 10 ImuMsg* from queue
//   3. give stoplight_cam  → cam thread sends 1 frame into queue
//   4. blocking-read 1 CamMsg* from queue
//   5. if IMU behind camera timestamp, repeat step 1-2
//   6. process camera frame, publish pose
// ==============================================================================
class OpenVINS_Plugin : public threadloop {
public:
    explicit OpenVINS_Plugin(phonebook_new& pb)
        : threadloop{pb, "openvins",
                     openvins_stack,
                     K_THREAD_STACK_SIZEOF(openvins_stack),
                     5}
        , vio_estimator_{nullptr}
        , imu_count_{0}
        , cam_count_{0}
        , update_count_{0}
        , latest_imu_t_{0.0}
    {
        printf("[OpenVINS] constructed (main thread).\n");
    }

    void _p_thread_setup() override {
        printf("[OpenVINS] _p_thread_setup() START  tid=%p\n", k_current_get());

        vio_config_ = create_vio_config();
        printf("[OpenVINS] VIOConfig done.\n");

        vio_estimator_ = new MSCKFEstimator(vio_config_);
        printf("[OpenVINS] MSCKFEstimator done. Entering loop.\n");
    }

    skip_option _p_should_skip() override {
        if (cam_count_ >= kExpectedCamFrames) {
            printf("[OpenVINS] all %u camera frames processed — stopping\n",
                   kExpectedCamFrames);
            return skip_option::stop;
        }
        return skip_option::run;
    }

    void _p_one_iteration() override {
        if (!vio_estimator_) return;

        uint32_t iter = cam_count_;

        // ── Step 1: release IMU, drain one window ────────────────────────
        printf("[OpenVINS] iter=%u  STEP1: giving stoplight_imu\n", iter);
        k_sem_give(&stoplight_imu);
        k_yield();

        printf("[OpenVINS] iter=%u  STEP1: waiting for %zu IMU msgs\n",
               iter, kImuSamplesPerWindow);

        for (size_t i = 0; i < kImuSamplesPerWindow; i++) {
            ImuMsg* imu_ptr = nullptr;
            int rc = k_msgq_get(&openvins_imu_queue, &imu_ptr, K_FOREVER);
            if (rc != 0 || !imu_ptr) {
                printf("[OpenVINS] iter=%u  ERROR: IMU get failed i=%zu rc=%d\n",
                       iter, i, rc);
                return;
            }
            process_imu(*imu_ptr);
            delete imu_ptr;
        }

        printf("[OpenVINS] iter=%u  STEP1: got %zu IMU, latest_imu_t=%.4f s\n",
               iter, kImuSamplesPerWindow, latest_imu_t_);

        // ── Step 2: release camera, read one frame ───────────────────────
        printf("[OpenVINS] iter=%u  STEP2: giving stoplight_cam\n", iter);
        k_sem_give(&stoplight_cam);
        k_yield();

        printf("[OpenVINS] iter=%u  STEP2: waiting for cam msg\n", iter);

        CamMsg* cam_ptr = nullptr;
        int rc = k_msgq_get(&openvins_cam_queue, &cam_ptr, K_FOREVER);
        if (rc != 0 || !cam_ptr) {
            printf("[OpenVINS] iter=%u  ERROR: cam get failed rc=%d ptr=%p\n",
                   iter, rc, cam_ptr);
            return;
        }

        double cam_t = static_cast<double>(
            cam_ptr->time.time_since_epoch().count()) * 1e-9;
        printf("[OpenVINS] iter=%u  STEP2: got cam frame t=%.4f s\n", iter, cam_t);

        // ── Step 3: ensure IMU covers camera timestamp ───────────────────
        while (cam_t > latest_imu_t_ + 1e-7) {
            printf("[OpenVINS] iter=%u  STEP3: IMU behind (%.4f < %.4f) — more IMU\n",
                   iter, latest_imu_t_, cam_t);

            k_sem_give(&stoplight_imu);
            k_yield();

            for (size_t i = 0; i < kImuSamplesPerWindow; i++) {
                ImuMsg* imu_ptr = nullptr;
                int rc2 = k_msgq_get(&openvins_imu_queue, &imu_ptr, K_FOREVER);
                if (rc2 != 0 || !imu_ptr) {
                    printf("[OpenVINS] iter=%u  ERROR: extra IMU get failed\n", iter);
                    delete cam_ptr;
                    return;
                }
                process_imu(*imu_ptr);
                delete imu_ptr;
            }
            printf("[OpenVINS] iter=%u  STEP3: latest_imu_t now %.4f s\n",
                   iter, latest_imu_t_);
        }

        // ── Step 4: process the camera frame ─────────────────────────────
        printf("[OpenVINS] iter=%u  STEP4: processing camera frame\n", iter);
        process_camera_frame(*cam_ptr);
        delete cam_ptr;

        printf("[OpenVINS] iter=%u  STEP4: done (cam_count=%u)\n", iter, cam_count_);
    }

    void stop() override {
        threadloop::stop();
        k_sem_give(&stoplight_imu);
        k_sem_give(&stoplight_cam);
        ImuMsg* dummy_imu = nullptr;
        CamMsg* dummy_cam = nullptr;
        k_msgq_put(&openvins_imu_queue, &dummy_imu, K_NO_WAIT);
        k_msgq_put(&openvins_cam_queue, &dummy_cam, K_NO_WAIT);
        printf("[OpenVINS] stop() called\n");
    }

    ~OpenVINS_Plugin() {
        ImuMsg* imu_ptr = nullptr;
        while (k_msgq_get(&openvins_imu_queue, &imu_ptr, K_NO_WAIT) == 0) {
            delete imu_ptr;
        }
        CamMsg* cam_ptr = nullptr;
        while (k_msgq_get(&openvins_cam_queue, &cam_ptr, K_NO_WAIT) == 0) {
            delete cam_ptr;
        }
        delete vio_estimator_;
    }

private:
    VIOConfig       vio_config_;
    MSCKFEstimator* vio_estimator_;

    uint32_t imu_count_;
    uint32_t cam_count_;
    uint32_t update_count_;
    double   latest_imu_t_;

    void process_imu(const ImuMsg& msg) {
        imu_count_++;
        double t = static_cast<double>(msg.time.time_since_epoch().count()) * 1e-9;
        printf("[OpenVINS] process_imu #%u  t=%.4f s\n", imu_count_, t);
        latest_imu_t_ = t;
        vio_estimator_->feed_imu(t, msg.angular_v, msg.linear_a);
    }

    void process_camera_frame(const CamMsg& msg) {
        cam_count_++;
        double t = static_cast<double>(msg.time.time_since_epoch().count()) * 1e-9;

        printf("[OpenVINS] cam #%u  feed_stereo() t=%.4f s  img0=%dx%d\n",
               cam_count_, t, msg.img0.cols, msg.img0.rows);

        if (msg.img0.empty() || msg.img1.empty()) {
            printf("[OpenVINS] ERROR: empty images — skipping\n");
            return;
        }
        if (msg.img0.type() != CV_8UC1 || msg.img1.type() != CV_8UC1) {
            printf("[OpenVINS] ERROR: expected CV_8UC1 (got %d/%d) — skipping\n",
                   msg.img0.type(), msg.img1.type());
            return;
        }

        vio_estimator_->feed_stereo(t, msg.img0, msg.img1);

        if (!vio_estimator_->is_initialized()) {
            printf("[OpenVINS] cam #%u: not yet initialized\n", cam_count_);
            return;
        }

        const IMUState& state = vio_estimator_->get_state();

        if (!std::isfinite(state.p_IinG.norm()) || !std::isfinite(state.q_GtoI.norm())) {
            printf("[OpenVINS] ERROR: non-finite state — skipping publish\n");
            return;
        }

        Eigen::Vector3f    pos_f  = state.p_IinG.cast<float>();
        Eigen::Quaternionf quat_f = state.q_GtoI.cast<float>();
        quat_f.normalize();

        if (!std::isfinite(quat_f.w()) || !std::isfinite(pos_f[0])) {
            printf("[OpenVINS] ERROR: non-finite after cast — skipping\n");
            return;
        }

        PoseMsg pose_msg{msg.time, pos_f, quat_f};
        node().publish_to<PoseMsg>("openvins_pose", pose_msg);

        ImuIntegratorInput integrator_msg{
            msg.time,
            duration{0},
            ImuParams{
                vio_config_.sigma_gyro,
                vio_config_.sigma_accel,
                vio_config_.sigma_gyro_bias,
                vio_config_.sigma_accel_bias,
                vio_config_.gravity,
                1.0,
                200.0
            },
            state.b_accel,
            state.b_gyro,
            state.p_IinG,
            state.v_IinG,
            state.q_GtoI
        };
        node().publish_to<ImuIntegratorInput>("imu_integrator", integrator_msg);

        update_count_++;

        printf(" [OpenVINS] UPDATE #%u  cam_t=%.4f s\n"
               "   pos  (m)   : [%.4f, %.4f, %.4f]\n"
               "   quat(wxyz) : [%.4f, %.4f, %.4f, %.4f]\n"
               "   vel  (m/s) : [%.4f, %.4f, %.4f]\n"
               "   clones     : %zu\n",
               update_count_, t,
               pos_f[0], pos_f[1], pos_f[2],
               quat_f.w(), quat_f.x(), quat_f.y(), quat_f.z(),
               state.v_IinG(0), state.v_IinG(1), state.v_IinG(2),
               state.clones.size());
    }
};

void start_openvins(phonebook_new& pb) {
    static OpenVINS_Plugin instance{pb};
    instance.start();
}

REGISTER_PLUGIN(openvins);