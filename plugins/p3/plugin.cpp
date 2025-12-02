// plugins/p3/plugin.cpp

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "../../src/node.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/data_format.hpp" // For ImuMsg definition

using namespace ILLIXR;

// -------------------------
// P3 (contains Node)
// -------------------------
class P3 {
public:
    P3(phonebook_new& pb)
        : node_{}
    {
        printk("[p3] Constructor\n");
        node_.initialize(pb, "p3");
    }

    void configure() {
        printk("[p3] configuring subscriptions\n");

        // Subscribing to ImuMsg from "offline_imu"
        node_.subscribe_from<ImuMsg>(
            "offline_imu", 
            &P3::on_imu_data, 
            this
        );
    }

    void start() {
        printk("[p3] start() thread running (Waiting for IMU data)\n");
        // We still keep the thread alive while callbacks handle the data.
        while (true) {
            k_sleep(K_SECONDS(1)); 
        }
    }

private:
    Node node_;

    // NEW CALLBACK: Handles incoming ImuMsg messages and prints data
    static void on_imu_data(void* ctx, const ImuMsg& msg) {
        // We don't need 'self' (ctx) for simple printing, but we keep the signature.
        (void)ctx; 
        
        // Access the time duration from the time_point
        long long time_ns = msg.time.time_since_epoch().count();

        printk("--- [p3] IMU DATA RECEIVED --- \n");
        printk("[p3] T_Relative: %lld ns\n", time_ns);
        
        // Print the Angular Velocity
        printk("[p3] Ang. Vel (rad/s): X: %f, Y: %f, Z: %f\n",
               msg.angular_v.x(), 
               msg.angular_v.y(), 
               msg.angular_v.z());
        
        // Print the Linear Acceleration
        printk("[p3] Lin. Acc (m/s^2): X: %f, Y: %f, Z: %f\n",
               msg.linear_a.x(), 
               msg.linear_a.y(), 
               msg.linear_a.z());
        printk("----------------------------------- \n");
    }
};

// -------------------------
// Factory
// -------------------------
void start_p3(phonebook_new& pb) {
    printk("[p3] start_p3() called\n");
    // IN the future, ready signal bit should be used to start the plugin
    static P3 instance{pb};
    instance.configure();
    // Note: Runtime will call instance.start() later if it uses the two-loop pattern.
    // If you need p3 to start immediately (like offline_imu), call instance.start() here.
}

REGISTER_PLUGIN(p3);