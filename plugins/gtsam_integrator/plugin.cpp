#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <atomic>  // For thread-safe flags
#include "../../src/plugin.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/data_format.hpp"
#include "third_party/filter.h"

using namespace ILLIXR;
LOG_MODULE_REGISTER(gtsam_int, LOG_LEVEL_INF);

#define GtsamInt_STACK_SIZE 1024
#define GtsamInt_PRIORITY   7

static K_THREAD_STACK_DEFINE(gtsam_int_stack_area, GtsamInt_STACK_SIZE);
K_MSGQ_DEFINE(gtsam_int_queue, sizeof(ImuMsg), 100, 4);
constexpr duration IMU_TTL{std::chrono::seconds{5}};

class GtsamIntPlugin : public Plugin {
public:
    GtsamIntPlugin(phonebook_new& pb) : Plugin(pb, "gtsam_int"), should_stop_{false} {
        // Subscribe to IMU data
        node_.subscribe_from<ImuMsg>(
            "offline_imu",
            [](void* ctx, const ImuMsg& msg) {
                // The Sender (IMU) thread runs this. 
                // It puts the data in our mailbox.
                k_msgq_put(&gtsam_int_queue, &msg, K_NO_WAIT);
            },
            this
        );
        for (int i = 0; i < 8; ++i) {
            filters.emplace_back(frequency, Eigen::Array<double, 3, 1>{mincutoff, mincutoff, mincutoff},
                                 Eigen::Array<double, 3, 1>{beta, beta, beta},
                                 Eigen::Array<double, 3, 1>{dcutoff, dcutoff, dcutoff}, Eigen::Array<double, 3, 1>::Zero(),
                                 Eigen::Array<double, 3, 1>::Ones(), [](auto& in) {
                                     return in.abs();
                                 });
        }

    }

    // Called when the system starts the plugin
    void start() override {
        should_stop_ = false; 
        k_thread_create(
            &thread_data_,
            gtsam_int_stack_area,
            K_THREAD_STACK_SIZEOF(gtsam_int_stack_area),
            thread_entry_point,
            this, NULL, NULL,
            K_PRIO_PREEMPT(GtsamInt_PRIORITY),
            0,
            K_NO_WAIT
        );
        
    }

    // Called when the system wants to shut down the plugin
    void stop() override {
        should_stop_ = true;
        
        // "Dummy Message" trick:
        // We send an empty message to wake up the thread immediately
        // so it doesn't stay asleep for the remaining 1-second timeout.
        ImuMsg wakeup_call{};
        k_msgq_put(&gtsam_int_queue, &wakeup_call, K_NO_WAIT);
        
        LOG_INF("[gtsam_int] Stop signal sent and thread nudged.");
    }

private:
    struct k_thread thread_data_;
    std::atomic<bool> should_stop_; // Prevents compiler optimization bugs
    unsigned int received_imu = 0;
    const double frequency = 200;
    const double mincutoff = 10;
    const double beta      = 1;
    const double dcutoff   = 10;
    std::vector<one_euro_filter<Eigen::Array<double, 3, 1>, double>> filters;
    Eigen::Matrix<double, 3, 1>                                      prev_euler_angles;
    bool                                                             has_prev = false;
    std::vector<ImuMsg> _imu_vec;


    static void thread_entry_point(void* p1, void*, void*) {
        static_cast<GtsamIntPlugin*>(p1)->run_loop();
    }

    void run_loop() {
        LOG_INF("[gtsam_int] Worker Thread Active: %p", k_current_get());
        LOG_INF("[gtsam_int] Printing the GTSAM filter data...");

        ImuMsg local_msg;

        while (!should_stop_) {
            // Wait for up to 1 second for a message
            int ret = k_msgq_get(&gtsam_int_queue, &local_msg, K_SECONDS(1));

            if (ret == 0) {
                // If we woke up because of the 'stop()' dummy message, exit now
                if (should_stop_) break;

                process_data(local_msg);
            } else {
                // This triggers if 1 second passes with an empty queue
                LOG_WRN("[gtsam_int] Idle: No IMU data for 1 second.");
            }
        }
        
        LOG_INF("[gtsam_int] Thread exiting clean. Total processed: %u", received_imu);
    }

    void process_data(const ImuMsg& msg) {

        _imu_vec.emplace_back(msg);
        received_imu++;

        clean_imu_vec(msg.time); 
        propagate_imu_values(msg.time);

    }
    void clean_imu_vec(time_point timestamp) {
        auto imu_iterator = _imu_vec.begin();

        // Since the vector is ordered oldest to latest, keep deleting until you
        // hit a value less than 'IMU_TTL' seconds old
        while (imu_iterator != _imu_vec.end()) {
            if (timestamp - imu_iterator->time < IMU_TTL) {
                break;
            }

            imu_iterator = _imu_vec.erase(imu_iterator);
        }
    }

    void propagate_imu_values(time_point timestamp) {

        LOG_INF("Propagate IMU values");
    }




};

void start_gtsam_integrator(phonebook_new& pb) {
    static GtsamIntPlugin instance{pb};
    instance.start();
}



REGISTER_PLUGIN(gtsam_integrator);