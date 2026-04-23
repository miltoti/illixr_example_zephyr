// plugins/imu_integrator/plugin.cpp
#include <zephyr/kernel.h>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "../../src/threadloop.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/data_format.hpp"

using namespace ILLIXR;

// offline_imu pushes ImuMsg* into this queue AND into openvins_imu_queue
K_MSGQ_DEFINE(imu_integrator_queue, sizeof(ImuMsg*), 500, 4);

K_THREAD_STACK_DEFINE(imu_integrator_stack, 65536);

class ImuIntegrator : public threadloop {
public:
    explicit ImuIntegrator(phonebook_new& pb)
        : threadloop{pb, "imu_integrator",
                     imu_integrator_stack,
                     K_THREAD_STACK_SIZEOF(imu_integrator_stack),
                     5}
        , has_state_{false}
        , last_t_{-1.0}
        , position_ {Eigen::Vector3d::Zero()}
        , velocity_ {Eigen::Vector3d::Zero()}
        , orientation_{Eigen::Quaterniond::Identity()}
        , bias_gyro_ {Eigen::Vector3d::Zero()}
        , bias_accel_{Eigen::Vector3d::Zero()}
        , gravity_   {0.0, 0.0, -9.81}
    {
        printf("[ImuIntegrator] constructed\n");
    }

    void _p_thread_setup() override {
        printf("[ImuIntegrator] setup tid=%p\n", k_current_get());

        // subscribe_from: static function pointer + void* context (this)
        // "openvins" is the sender name (matches REGISTER_PLUGIN(openvins))
        node().subscribe_from<ImuIntegratorInput>(
            "openvins",
            &ImuIntegrator::on_vio_state_cb,
            this);
    }

    skip_option _p_should_skip() override {
        if (!has_state_)
            return skip_option::skip_and_yield;
        if (k_msgq_num_used_get(&imu_integrator_queue) == 0)
            return skip_option::skip_and_yield;
        return skip_option::run;
    }

    void _p_one_iteration() override {
        ImuMsg* imu = nullptr;
        if (k_msgq_get(&imu_integrator_queue, &imu, K_NO_WAIT) != 0 || !imu)
            return;

        double t = static_cast<double>(
            imu->time.time_since_epoch().count()) * 1e-9;

        double dt = (last_t_ < 0.0) ? 0.0 : (t - last_t_);
        last_t_ = t;

        if (dt <= 0.0 || dt > 0.1) {
            delete imu;
            return;
        }

        // ── Remove biases ─────────────────────────────────────────────────
        Eigen::Vector3d gyro  = imu->angular_v - bias_gyro_;
        Eigen::Vector3d accel = imu->linear_a  - bias_accel_;
        delete imu;

        // ── Rotate accel IMU → global frame ──────────────────────────────
        // orientation_ = q_GtoI  →  R_ItoG = R_GtoI^T
        Eigen::Matrix3d R_ItoG = orientation_.toRotationMatrix().transpose();
        Eigen::Vector3d accel_global = R_ItoG * accel;

        // ── Remove gravity ────────────────────────────────────────────────
        // gravity_ = {0,0,-9.81}: subtract to get true acceleration
        Eigen::Vector3d accel_true = accel_global - gravity_;

        // ── Integrate position + velocity (Euler) ─────────────────────────
        position_ += velocity_ * dt + 0.5 * accel_true * dt * dt;
        velocity_ += accel_true * dt;

        // ── Integrate orientation ─────────────────────────────────────────
        // gyro in IMU frame, right-multiply onto q_GtoI
        double angle = gyro.norm() * dt;
        if (angle > 1e-10) {
            Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, gyro.normalized()));
            orientation_ = (orientation_ * dq).normalized();
        }

        // ── Publish ───────────────────────────────────────────────────────
        PoseMsg out{
            ILLIXR::time_point{std::chrono::nanoseconds{
                static_cast<long long>(t * 1e9)}},
            position_.cast<float>(),
            orientation_.cast<float>().normalized()
        };
        // "renderer" is the receiver name (matches REGISTER_PLUGIN(renderer))
        node().publish_to<PoseMsg>("renderer", out);

        printf("[ImuIntegrator] t=%.4f  pos=[%.3f,%.3f,%.3f]\n",
               t, position_.x(), position_.y(), position_.z());
    }

private:
    bool   has_state_;
    double last_t_;

    // Integration state — exact types from ImuIntegratorInput
    Eigen::Vector3d    position_;     // p_IinG  metres, global frame
    Eigen::Vector3d    velocity_;     // v_IinG  m/s,    global frame
    Eigen::Quaterniond orientation_;  // q_GtoI  global → IMU
    Eigen::Vector3d    bias_gyro_;    // rad/s
    Eigen::Vector3d    bias_accel_;   // m/s²
    Eigen::Vector3d    gravity_;      // {0,0,-9.81} m/s²

    // Static callback required by subscribe_from API
    static void on_vio_state_cb(void* ctx, const ImuIntegratorInput& msg) {
        static_cast<ImuIntegrator*>(ctx)->on_vio_state(msg);
    }

    void on_vio_state(const ImuIntegratorInput& msg) {
        // Reset integration baseline to authoritative VIO state
        // Field mapping from ImuIntegratorInput (data_format.hpp):
        position_    = msg.position;            // p_IinG  Vector3d
        velocity_    = msg.velocity;            // v_IinG  Vector3d
        orientation_ = msg.orientation.normalized(); // q_GtoI Quaterniond
        bias_gyro_   = msg.bias_gyro;           // Vector3d
        bias_accel_  = msg.bias_accel;          // Vector3d
        gravity_     = msg.params.n_gravity;    // {0,0,-9.81} Vector3d

        last_t_ = static_cast<double>(
            msg.timestamp.time_since_epoch().count()) * 1e-9;
        has_state_ = true;

        printf("[ImuIntegrator] VIO reset  t=%.4f"
               "  pos=[%.3f,%.3f,%.3f]  vel=[%.3f,%.3f,%.3f]\n",
               last_t_,
               position_.x(), position_.y(), position_.z(),
               velocity_.x(), velocity_.y(), velocity_.z());
    }
};

void start_imu_integrator(phonebook_new& pb) {
    static ImuIntegrator instance{pb};
    instance.start();
}

REGISTER_PLUGIN(imu_integrator);