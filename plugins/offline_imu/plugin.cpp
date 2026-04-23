#include <zephyr/kernel.h>
#include <cstdint>
#include <chrono>
#include <cmath>

#include "../../src/threadloop.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/data_format.hpp"
#include "../../src/stoplight.hpp"
#include "../openvins/openvins_queues.hpp"
#include "../imu_integrator/imu_integrator_queue.hpp"

#include "embedded_imu.hpp"

using namespace ILLIXR;

extern uint64_t g_program_start_mtime;

// CLINT mtime: global real-time counter, consistent across all harts
static inline uint64_t read_mtime() {
    volatile uint64_t* mtime = reinterpret_cast<volatile uint64_t*>(0x200bff8UL);
    return *mtime;
}

K_THREAD_STACK_DEFINE(offline_imu_stack, 262144);

// Window size must match what openvins expects
static constexpr size_t kSamplesPerWindow = 10;

class Offline_imu : public threadloop {
public:
    explicit Offline_imu(phonebook_new& pb)
        : threadloop{pb, "offline_imu",
                     offline_imu_stack,
                     K_THREAD_STACK_SIZEOF(offline_imu_stack),
                     5}
        , current_idx_{0}
    {
        printf("[offline_imu] constructed  samples=%zu (EuRoC embedded)\n",
               kEmbeddedImuCount);
    }

    skip_option _p_should_skip() override {
        if (current_idx_ >= kEmbeddedImuCount)
            return skip_option::stop;
        return skip_option::run;
    }

    void _p_one_iteration() override {
        size_t pos_in_window = current_idx_ % kSamplesPerWindow;

        if (pos_in_window == 0) {
            k_sem_take(&stoplight_imu, K_FOREVER);
            printf("[Offline_imu] Green light received, sending window starting at #%zu\n",
                   current_idx_);
        }

        const auto& s = kEmbeddedImu[current_idx_];

        ImuMsg* msg_vins = new ImuMsg{
            time_point{std::chrono::nanoseconds{s.ts_ns}},
            Eigen::Vector3d{s.wx, s.wy, s.wz},
            Eigen::Vector3d{s.ax, s.ay, s.az}
        };
        ImuMsg* msg_int = new ImuMsg{*msg_vins};  // copy

        int rc1 = k_msgq_put(&openvins_imu_queue,     &msg_vins, K_NO_WAIT);
        int rc2 = k_msgq_put(&imu_integrator_queue,   &msg_int,  K_NO_WAIT);

        if (rc1 != 0) { delete msg_vins; printf("[offline_imu] ERROR: openvins queue full\n"); }
        if (rc2 != 0) { delete msg_int;  printf("[offline_imu] ERROR: integrator queue full\n"); }
        printf("[offline_imu] IMU #%04zu ts=%lld ns\n",
               current_idx_ + 1, (long long)s.ts_ns);

        ++current_idx_;

        if (current_idx_ == 50) {
            uint64_t end_mtime  = read_mtime();
            uint64_t elapsed    = end_mtime - g_program_start_mtime;
            // CLINT mtime ticks at the CPU reference clock (10 MHz on Spike by default)
            // elapsed / 10_000_000 = wall seconds
            double   elapsed_s  = static_cast<double>(elapsed) / 10000000.0;
            double   imu_span_s = static_cast<double>(s.ts_ns - kEmbeddedImu[0].ts_ns) * 1e-9;
            printf("\n[MTIME_COUNT] 50 IMU samples processed (global CLINT mtime)\n");
            printf("[MTIME_COUNT]   start  mtime  : %llu ticks\n", (unsigned long long)g_program_start_mtime);
            printf("[MTIME_COUNT]   end    mtime  : %llu ticks\n", (unsigned long long)end_mtime);
            printf("[MTIME_COUNT]   elapsed ticks : %llu\n",       (unsigned long long)elapsed);
            printf("[MTIME_COUNT]   elapsed time  : %.6f s  (at 10 MHz mtime clock)\n", elapsed_s);
            printf("[MTIME_COUNT]   IMU data span : %.6f s\n", imu_span_s);
            printf("[MTIME_COUNT]   slowdown ratio: %.2fx real-time\n\n", elapsed_s / imu_span_s);
        }
        k_yield();
    }

private:
    size_t current_idx_;
};

void start_offline_imu(phonebook_new& pb) {
    static Offline_imu instance{pb};
    instance.start();
}

REGISTER_PLUGIN(offline_imu);