// plugins/renderer/plugin.cpp
#include <zephyr/kernel.h>
#include <cstdio>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <vector>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../../src/threadloop.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/data_format.hpp"
#include "../../src/data_format_opencv.hpp"
#include "../offline_cam/embedded_cam.hpp"   // kEmbeddedCam[], kEmbeddedCamCount

using namespace ILLIXR;

static constexpr char  kOutputDir[]  = "output/";
static constexpr float kTrajScale    = 80.0f;
static constexpr int   kTrajMapSize  = 160;

K_THREAD_STACK_DEFINE(renderer_stack, 131072);

// ── Utility: q_GtoI → roll/pitch/yaw (ZYX, radians) ─────────────────────────
// Invert q_GtoI → q_ItoG first so we get IMU orientation in global frame
static void quat_to_rpy(const Eigen::Quaternionf& q_GtoI,
                        float& roll, float& pitch, float& yaw)
{
    Eigen::Quaternionf q = q_GtoI.conjugate().normalized();
    float w = q.w(), x = q.x(), y = q.y(), z = q.z();

    float sinr_cosp = 2.f * (w*x + y*z);
    float cosr_cosp = 1.f - 2.f * (x*x + y*y);
    roll  = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.f * (w*y - z*x);
    pitch = (std::fabs(sinp) >= 1.f)
            ? std::copysign((float)M_PI / 2.f, sinp)
            : std::asin(sinp);

    float siny_cosp = 2.f * (w*z + x*y);
    float cosy_cosp = 1.f - 2.f * (y*y + z*z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// ── Project a global-frame unit axis into 2D image plane ─────────────────────
// Apply q_GtoI to rotate global → IMU/camera frame, then project (x right, y down)
static cv::Point2f project_world_axis(const Eigen::Quaternionf& q_GtoI,
                                      const Eigen::Vector3f& axis_global)
{
    Eigen::Vector3f v = q_GtoI * axis_global;
    return cv::Point2f(v.x(), -v.y());  // flip y for screen convention
}

static void draw_axis(cv::Mat& img, cv::Point2f origin, cv::Point2f dir,
                      cv::Scalar color, const char* label)
{
    cv::Point2f tip = origin + dir * 55.f;
    cv::arrowedLine(img, origin, tip, color, 2, cv::LINE_AA, 0, 0.2);
    cv::putText(img, label, tip + cv::Point2f(4.f, 4.f),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
}

// ==============================================================================
class Renderer : public threadloop {
public:
    explicit Renderer(phonebook_new& pb)
        : threadloop{pb, "renderer",
                     renderer_stack,
                     K_THREAD_STACK_SIZEOF(renderer_stack),
                     5}
        , frame_idx_{0}
        , has_pose_{false}
        , fs_failed_{false}
    {
        printf("[Renderer] constructed\n");
        printf("[Renderer]   output dir : %s\n", kOutputDir);
        printf("[Renderer]   total frames: %zu\n", kEmbeddedCamCount);
        printf("[Renderer]   NOTE: imwrite will fail on Spike/Zephyr — no host FS mounted.\n");
        printf("[Renderer]         Rendering proceeds; pose/orientation data logged per frame.\n");
    }

    void _p_thread_setup() override {
        printf("[Renderer] setup tid=%p\n", k_current_get());

        // subscribe_from: static callback + void* context
        // "imu_integrator" is the sender (matches REGISTER_PLUGIN(imu_integrator))
        node().subscribe_from<PoseMsg>(
            "imu_integrator",
            &Renderer::on_pose_cb,
            this);
    }

    skip_option _p_should_skip() override {
        if (frame_idx_ >= kEmbeddedCamCount)
            return skip_option::stop;
        if (!has_pose_)
            return skip_option::skip_and_yield;
        return skip_option::run;
    }

    void _p_one_iteration() override {
        PoseMsg pose = latest_pose_;
        has_pose_ = false;
        render_frame(pose, frame_idx_);
        ++frame_idx_;
    }

private:
    uint32_t frame_idx_;
    bool     has_pose_;
    bool     fs_failed_;   // suppress repeated FS error banners after first failure
    PoseMsg  latest_pose_;

    // Trajectory history: (x, z) in global frame for top-down map
    std::vector<cv::Point2f> traj_;

    // Static callback required by subscribe_from API
    static void on_pose_cb(void* ctx, const PoseMsg& msg) {
        static_cast<Renderer*>(ctx)->on_pose(msg);
    }

    void on_pose(const PoseMsg& msg) {
        latest_pose_ = msg;
        has_pose_    = true;
    }

    void render_frame(const PoseMsg& pose, uint32_t idx) {

        // ── 1. Decode cam0 from embedded PNG bytes ────────────────────────
        const auto& frame = kEmbeddedCam[idx];
        cv::Mat buf(1, (int)frame.cam0_size, CV_8UC1,
                    const_cast<uint8_t*>(frame.cam0_png));
        cv::Mat gray = cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
        if (gray.empty()) {
            printf("[Renderer] ERROR frame %u: imdecode returned empty Mat\n", idx);
            printf("[Renderer]   cam0_png ptr=%p  cam0_size=%zu bytes  ts=%lld ns\n",
                   (const void*)frame.cam0_png, frame.cam0_size,
                   (long long)frame.ts_ns);
            printf("[Renderer]   Likely cause: embedded PNG bytes corrupt or imgcodecs "
                   "not built with PNG support.\n");
            return;
        }

        // ── 2. Grayscale → BGR for colour overlays ────────────────────────
        cv::Mat canvas;
        cv::cvtColor(gray, canvas, cv::COLOR_GRAY2BGR);

        // ── 3. Extract pose ───────────────────────────────────────────────
        // position    = p_IinG  (Vector3f, metres, global frame)
        // orientation = q_GtoI  (Quaternionf, global → IMU)
        const float px = pose.position.x();
        const float py = pose.position.y();
        const float pz = pose.position.z();
        const Eigen::Quaternionf& q_GtoI = pose.orientation;

        float roll, pitch, yaw;
        quat_to_rpy(q_GtoI, roll, pitch, yaw);
        constexpr float kR2D = 180.f / M_PI;

        double ts = static_cast<double>(
            pose.timestamp.time_since_epoch().count()) * 1e-9;

        // ── 4. Dark banner for text ───────────────────────────────────────
        {
            cv::Mat overlay = canvas.clone();
            cv::rectangle(overlay, {0, 0}, {canvas.cols, 95},
                          cv::Scalar(0, 0, 0), cv::FILLED);
            cv::addWeighted(overlay, 0.6, canvas, 0.4, 0.0, canvas);
        }

        // ── 5. Text overlays ──────────────────────────────────────────────
        auto put = [&](const char* text, int row, cv::Scalar col) {
            cv::putText(canvas, text, cv::Point(10, 18 + row * 22),
                        cv::FONT_HERSHEY_SIMPLEX, 0.52, col, 1, cv::LINE_AA);
        };

        char tbuf[128];
        snprintf(tbuf, sizeof(tbuf), "Frame %03u   t = %.4f s", idx, ts);
        put(tbuf, 0, cv::Scalar(255, 255, 255));

        snprintf(tbuf, sizeof(tbuf),
                 "Pos (m)  x=%.3f  y=%.3f  z=%.3f", px, py, pz);
        put(tbuf, 1, cv::Scalar(80, 255, 80));

        snprintf(tbuf, sizeof(tbuf),
                 "RPY(deg) r=%.1f  p=%.1f  y=%.1f",
                 roll*kR2D, pitch*kR2D, yaw*kR2D);
        put(tbuf, 2, cv::Scalar(80, 200, 255));

        snprintf(tbuf, sizeof(tbuf),
                 "q_GtoI  w=%.3f x=%.3f y=%.3f z=%.3f",
                 q_GtoI.w(), q_GtoI.x(), q_GtoI.y(), q_GtoI.z());
        put(tbuf, 3, cv::Scalar(160, 160, 160));

        // ── 6. Orientation axes at image centre ───────────────────────────
        cv::Point2f centre(canvas.cols * 0.5f, canvas.rows * 0.58f);

        cv::Point2f ax_x = project_world_axis(q_GtoI, Eigen::Vector3f::UnitX());
        cv::Point2f ax_y = project_world_axis(q_GtoI, Eigen::Vector3f::UnitY());
        cv::Point2f ax_z = project_world_axis(q_GtoI, Eigen::Vector3f::UnitZ());

        draw_axis(canvas, centre, ax_x, cv::Scalar(0,   0,   255), "X");
        draw_axis(canvas, centre, ax_y, cv::Scalar(0,   220,   0), "Y");
        draw_axis(canvas, centre, ax_z, cv::Scalar(255, 120,   0), "Z");

        // ── 7. Top-down trajectory mini-map (X-Z plane) ───────────────────
        traj_.push_back(cv::Point2f(px, pz));

        int mx0 = canvas.cols - kTrajMapSize - 8;
        int my0 = canvas.rows - kTrajMapSize - 8;
        cv::Mat map_bg(kTrajMapSize, kTrajMapSize,
                       CV_8UC3, cv::Scalar(15, 15, 15));

        for (int g = 0; g < kTrajMapSize; g += 40) {
            cv::line(map_bg, {g, 0}, {g, kTrajMapSize}, {45,45,45}, 1);
            cv::line(map_bg, {0, g}, {kTrajMapSize, g}, {45,45,45}, 1);
        }

        int mc = kTrajMapSize / 2;
        cv::line(map_bg, {mc-8, mc}, {mc+8, mc}, {80,80,80}, 1);
        cv::line(map_bg, {mc, mc-8}, {mc, mc+8}, {80,80,80}, 1);

        auto to_px = [&](cv::Point2f p) -> cv::Point {
            return { mc + (int)(p.x * kTrajScale),
                     mc - (int)(p.y * kTrajScale) };
        };

        for (size_t i = 1; i < traj_.size(); ++i)
            cv::line(map_bg, to_px(traj_[i-1]), to_px(traj_[i]),
                     {0, 200, 100}, 1, cv::LINE_AA);

        cv::circle(map_bg, to_px(traj_.back()),
                   4, cv::Scalar(0, 60, 255), -1, cv::LINE_AA);

        cv::putText(map_bg, "TOP-DOWN XZ", {3, 11},
                    cv::FONT_HERSHEY_SIMPLEX, 0.3,
                    cv::Scalar(130,130,130), 1, cv::LINE_AA);

        cv::Rect roi(mx0, my0, kTrajMapSize, kTrajMapSize);
        if (roi.x >= 0 && roi.y >= 0 &&
            roi.x + roi.width  <= canvas.cols &&
            roi.y + roi.height <= canvas.rows) {
            cv::addWeighted(map_bg, 0.9, canvas(roi), 0.1, 0.0, canvas(roi));
        }

        // ── 8. Border ─────────────────────────────────────────────────────
        cv::rectangle(canvas, {0, 0},
                      {canvas.cols-1, canvas.rows-1},
                      cv::Scalar(0, 180, 255), 2);

        // ── 9. Save PNG ───────────────────────────────────────────────────
        char path[128];
        snprintf(path, sizeof(path), "%srender_%04u.png", kOutputDir, idx);
        errno = 0;
        bool ok = cv::imwrite(path, canvas);

        // Always print the pose/orientation result regardless of FS outcome
        printf("[Renderer] frame=%03u  t=%.4f s\n", idx, ts);
        printf("[Renderer]   pos (m)  : [%.4f, %.4f, %.4f]\n", px, py, pz);
        printf("[Renderer]   rpy (deg): [%.2f, %.2f, %.2f]\n",
               roll*kR2D, pitch*kR2D, yaw*kR2D);
        printf("[Renderer]   q_GtoI   : w=%.4f x=%.4f y=%.4f z=%.4f\n",
               q_GtoI.w(), q_GtoI.x(), q_GtoI.y(), q_GtoI.z());
        printf("[Renderer]   canvas   : %dx%d  path: %s\n",
               canvas.cols, canvas.rows, path);

        if (ok) {
            printf("[Renderer]   imwrite  : OK\n");
        } else {
            int saved_errno = errno;
            if (!fs_failed_) {
                printf("[Renderer]   imwrite  : FAILED — first failure, full diagnostics:\n");
                printf("[Renderer]     errno=%d (%s)\n",
                       saved_errno, saved_errno ? strerror(saved_errno) : "no errno set");
                printf("[Renderer]     Path attempted : \"%s\"\n", path);
                printf("[Renderer]     Likely cause   : output directory \"%s\" does not exist\n",
                       kOutputDir);
                printf("[Renderer]     on Spike/Zephyr no host filesystem is mounted — "
                       "imwrite cannot write files.\n");
                printf("[Renderer]     Rendering is still computed correctly; "
                       "pose data above is valid.\n");
                fs_failed_ = true;
            } else {
                printf("[Renderer]   imwrite  : FAILED (errno=%d, FS not mounted — see frame 0 for details)\n",
                       saved_errno);
            }
        }
    }
};

void start_renderer(phonebook_new& pb) {
    static Renderer instance{pb};
    instance.start();
}

REGISTER_PLUGIN(renderer);