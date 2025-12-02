// plugins/offline_imu/plugin.cpp
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <map>
#include <cstdint>
#include <chrono>

// Core ILLIXR includes
#include "../../src/plugin.hpp" // <-- Now inheriting this
#include "../../src/node.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/data_format.hpp" 
#include "../../src/relative_clock.hpp" 

// Plugin-specific data loading
#include "data_loading.hpp" 

using namespace ILLIXR;

// ---------------------------------------------------------
// Plugin: Offline_imu (Inherits Plugin)
// ---------------------------------------------------------
class Offline_imu : public Plugin {
public:
    explicit Offline_imu(phonebook_new& pb)
        // Initialize base class (Plugin) which calls node().initialize(pb, "offline_imu")
        : Plugin{pb, "offline_imu"}
        // Use the global accessor to initialize the clock reference
        , clock_{get_global_relative_clock()}
        , _m_sensor_data{load_data()}
        , _m_sensor_data_it{_m_sensor_data.cbegin()}
        // Store the first timestamp, checking if data is empty
        , dataset_first_time{_m_sensor_data_it != _m_sensor_data.end() ? _m_sensor_data_it->first : 0}
    {
        printk("[Offline_imu] Constructor\n");
        // Node is initialized by the base class constructor.
    }

    // Must be virtual if you intend to use the base class pointer
    virtual void configure() {
        // No setup needed here for the single-thread model.
    }

    // Node::start() is the core logic, executed by the thread launched by the Runtime.
    // This function BLOCKS execution until all data is published.
    virtual void start() {
        if (_m_sensor_data.empty()) {
            printk("[offline_imu] No IMU data loaded. Playback skipped.\n");
            return;
        }
        
        printk("[offline_imu] Starting single-thread time-synchronized playback...\n");
        clock_.start(); // Crucial: Start the global clock for time comparison
        
        // Execute the loop directly in this thread
        run_data_playback_loop();
        
        printk("[offline_imu] Plugin main thread finished execution.\n");
    }

private:
    // --- Data and Clock Members (Composition Pattern) ---
    // Note: Node is now inherited from the Plugin base class.
    RelativeClock&                                 clock_;
    const std::map<ullong, sensor_types>           _m_sensor_data;
    std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
    ullong                                         dataset_first_time;
    
    // --- Core Playback Loop ---
    void run_data_playback_loop() {
        while (_m_sensor_data_it != _m_sensor_data.end()) {
            
            ullong dataset_now = _m_sensor_data_it->first;

            // Calculate the required elapsed time from the dataset's start
            auto dataset_elapsed_time = std::chrono::nanoseconds{dataset_now - dataset_first_time};
            
            // Get the current elapsed time of the ILLIXR Relative Clock
            auto rtc_elapsed_time = clock_.now().time_since_epoch();

            // Check if the current clock time has reached or passed the next data point's timestamp
            if (rtc_elapsed_time >= dataset_elapsed_time) {
                
                const sensor_types& sensor_datum = _m_sensor_data_it->second;

                // 1. CONSTRUCT THE IMU MESSAGE using ILLIXR::ImuMsg
                ImuMsg imu_msg {
                    // CRITICAL: Construct time_point from the duration
                    time_point{dataset_elapsed_time}, 
                    // Angular Velocity (Eigen::Vector3d)
                    sensor_datum.imu0.angular_v,
                    // Linear Acceleration (Eigen::Vector3d)
                    sensor_datum.imu0.linear_a
                };

                // 2. PUBLISH TO P3 - using node() accessor from the Plugin base class
                node().publish_to<ImuMsg>("p3", imu_msg); 
                
                printk("[offline_imu] [TIME HIT] PUBLISHED ImuMsg to p3 at T_rel=%lld ns\n",
                       (long long)dataset_elapsed_time.count());
                
                // Move to the next data point
                ++_m_sensor_data_it;
            } else {
                // Time hasn't arrived. Sleep briefly to yield the CPU.
                k_usleep(1); 
            }
        }
    }
};

// ---------------------------------------------------------
// Factory
// ---------------------------------------------------------
void start_offline_imu(phonebook_new& pb) {
    // IN the future, ready signal bit should be used to start the plugin

    printk("[offline_imu] start_offline_imu() called\n");
    // 1. Construct the instance
    static Offline_imu instance{pb};
    
    // 2. CRITICAL: The k_sleep(K_SECONDS(10)) and instance.start() were here 
    // in the version that caused the race condition. We keep the logic as 
    // originally requested to be "working" (i.e., reproducing the problem).
    // Delay for 10 seconds
    k_sleep(K_SECONDS(10));
    instance.start(); 
}

REGISTER_PLUGIN(offline_imu);