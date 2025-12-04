#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <map>
#include <cstdint>
#include <chrono>

// Core ILLIXR includes
#include "../../src/plugin.hpp"
#include "../../src/node.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/data_format.hpp"
#include "../../src/relative_clock.hpp"

#include "data_loading.hpp"

using namespace ILLIXR;

// ==============================================================================
// THREAD STACK DEFINITION
// ==============================================================================
#define IMU_STACK_SIZE 4096
#define IMU_PRIORITY   5

static K_THREAD_STACK_DEFINE(imu_stack_area, IMU_STACK_SIZE);

// ==============================================================================
// PLUGIN CLASS
// ==============================================================================
class Offline_imu : public Plugin {
public:
    explicit Offline_imu(phonebook_new& pb)
        : Plugin{pb, "offline_imu"}
        , clock_{get_global_relative_clock()}
        , _m_sensor_data{load_data()}
        , _m_sensor_data_it{_m_sensor_data.cbegin()}
        , dataset_first_time{
              _m_sensor_data_it != _m_sensor_data.end()
                  ? _m_sensor_data_it->first
                  : 0
          }
    {
        printk("[offline_imu] Constructor. Data size: %zu\n", _m_sensor_data.size());

        node().publish_to_periodic<int>(
            "p3",
            1000,
            []() -> int {
                static int value = 0;
                return value++;
            }
        );
    }

    virtual void start() override {
        if (_m_sensor_data.empty()) {
            printk("[offline_imu] No IMU data loaded. Thread skipped.\n");
            return;
        }

        printk("[offline_imu] Spawning playback thread...\n");

        k_thread_create(
            &thread_data_,
            imu_stack_area,
            K_THREAD_STACK_SIZEOF(imu_stack_area),
            thread_entry_point,
            this, NULL, NULL,
            K_PRIO_PREEMPT(IMU_PRIORITY),
            0,
            K_NO_WAIT
        );
    }

private:
    RelativeClock&                                 clock_;
    const std::map<ullong, sensor_types>           _m_sensor_data;
    std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
    ullong                                         dataset_first_time;

    struct k_thread thread_data_;

    static void thread_entry_point(void* p1, void*, void*) {
        Offline_imu* self = static_cast<Offline_imu*>(p1);
        self->run_data_playback_loop();
    }

    void run_data_playback_loop() {
        printk("[offline_imu] Thread running (ID: %p). Starting clock.\n", k_current_get());
        clock_.start(); 

        while (!should_stop_) {
            node_.service_periodic();

            if (_m_sensor_data_it != _m_sensor_data.end()) {
                ullong dataset_now = _m_sensor_data_it->first;
                auto dataset_elapsed = std::chrono::nanoseconds{dataset_now - dataset_first_time};
                auto rtc_elapsed     = clock_.now().time_since_epoch();

                if (rtc_elapsed >= dataset_elapsed) {
                    const sensor_types& sensor_datum = _m_sensor_data_it->second;

                    ImuMsg imu_msg{
                        time_point{dataset_elapsed},
                        sensor_datum.imu0.angular_v,
                        sensor_datum.imu0.linear_a
                    };

                    node().publish_to<ImuMsg>("p3", imu_msg);

                    // DEBUG: Print EVERY message to confirm flow
                    static int count = 0;
                    if (++count % 1 == 0) {
                    
                         printk("[offline_imu] Push T=%lld\n", (long long)dataset_elapsed.count());
                         // Thread information
                         printk("[offline_imu]   Thread ID: %p\n", k_current_get());
                    
                    }
                    ++_m_sensor_data_it;
                } 
            } else {
                static bool printed_end = false;
                if (!printed_end) {
                    printk("[offline_imu] End of dataset reached.\n");
                    printed_end = true;
                }
                k_msleep(10);
                continue;
            }

            k_usleep(100);
        }
    }
};

void start_offline_imu(phonebook_new& pb) {
    static Offline_imu instance{pb};
    instance.start();
}

REGISTER_PLUGIN(offline_imu);