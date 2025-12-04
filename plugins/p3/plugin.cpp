#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "../../src/plugin.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/data_format.hpp"

using namespace ILLIXR;

#define P3_STACK_SIZE 1024
#define P3_PRIORITY   7

static K_THREAD_STACK_DEFINE(p3_stack_area, P3_STACK_SIZE);
K_MSGQ_DEFINE(p3_queue, sizeof(ImuMsg), 10, 4);

class P3Plugin : public Plugin {
public:
    P3Plugin(phonebook_new& pb) : Plugin(pb, "p3") {
        // We only subscribe to IMU data now.
        // We handle it Asynchronously via the Queue.
        node_.subscribe_from<ImuMsg>(
            "offline_imu",
            [](void* ctx, const ImuMsg& msg) {
                // Runs on SENDER Thread (IMU Thread)
                // Action: Quick hand-off to the mailbox.
                k_msgq_put(&p3_queue, &msg, K_NO_WAIT);
            },
            this
        );
    }

    void start() override {
        k_thread_create(
            &thread_data_,
            p3_stack_area,
            K_THREAD_STACK_SIZEOF(p3_stack_area),
            thread_entry_point,
            this, NULL, NULL,
            K_PRIO_PREEMPT(P3_PRIORITY),
            0,
            K_NO_WAIT
        );
    }

private:
    struct k_thread thread_data_;

    static void thread_entry_point(void* p1, void*, void*) {
        static_cast<P3Plugin*>(p1)->run_loop();
    }

    void run_loop() {
        // Print identity ONCE
        printk("========================================\n");
        printk("[p3] WORKER STARTED. Thread ID: %p\n", k_current_get());
        printk("========================================\n");
        
        ImuMsg local_msg;

        while (!should_stop_) {
            node_.service_periodic();

            // Wait for data (blocks here until IMU sends something)
            if (k_msgq_get(&p3_queue, &local_msg, K_MSEC(20)) == 0) {
                process_data(local_msg);
            }
        }
    }

    void process_data(const ImuMsg& msg) {
        // Runs on RECEIVER Thread (P3 Thread)
        printk("[p3] Processing time: %lld | Thread: %p\n", 
               (long long)msg.time.time_since_epoch().count(),
               k_current_get());
        // Print data
        // EIgen format
        printk("[p3]   Angular Vel: [%f, %f, %f]\n",
               msg.angular_v[0], msg.angular_v[1], msg.angular_v[2]);
        printk("[p3]   Linear Acc: [%f, %f, %f]\n",
               msg.linear_a[0], msg.linear_a[1], msg.linear_a[2]);
    }
};

void start_p3(phonebook_new& pb) {
    static P3Plugin instance{pb};
    instance.start();
}

REGISTER_PLUGIN(p3);