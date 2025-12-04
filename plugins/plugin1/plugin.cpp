// plugins/plugin1/plugin.cpp

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "../../src/plugin.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/relative_clock.hpp"

using namespace ILLIXR;

// ---------- static Zephyr thread objects for this plugin ----------
static K_THREAD_STACK_DEFINE(plugin1_stack, 4096);
static struct k_thread       plugin1_thread;

// ---------- message type ----------
struct SensorMsg {
    std::int64_t t_ns;  // timestamp in nanoseconds
};

class Plugin1 : public Plugin {
public:
    explicit Plugin1(phonebook_new& pb)
        : Plugin{pb, "plugin1"}
        , clock_{get_global_relative_clock()}
        , counter_{0}
    {
        printk("[plugin1] ctor\n");
        configure();
    }

    void configure() {
        printk("[plugin1] configuring periodic publisher\n");

        // Register a periodic job on our Node.
        // No extra thread here; Node just stores the job.
        node().publish_to_periodic<SensorMsg>(
            "plugin2",
            500,   // every 500 ms
            [this]() -> SensorMsg {
                std::int64_t t_ns = clock_.now_ns();

                printk("[plugin1] sending sample #%d at t=%lld ns\n",
                       counter_, (long long)t_ns);

                ++counter_;
                return SensorMsg{t_ns};
            }
        );
    }

    // --------- plugin worker loop (runs in plugin1 thread) ---------
    void thread_loop() {
        printk("[plugin1] thread_loop start\n");

        while (true) {
            // Drive all periodic jobs registered via publish_to_periodic()
            node().service_periodic();

            // Do any other plugin1 work here...

            k_msleep(10);  // don’t busy spin
        }
    }

    // Static Zephyr entry trampoline
    static void thread_entry(void* p1, void*, void*) {
        auto* self = static_cast<Plugin1*>(p1);
        self->thread_loop();
    }

    // --------- required by Plugin base: spawn thread, return immediately ---------
    void start() override {
        printk("[plugin1] Spawning thread...\n");

        k_tid_t tid = k_thread_create(
            &plugin1_thread,
            plugin1_stack,
            K_THREAD_STACK_SIZEOF(plugin1_stack),
            &Plugin1::thread_entry,
            this, nullptr, nullptr,
            5,  // priority
            0,
            K_NO_WAIT
        );

        k_thread_name_set(tid, "plugin1");
        printk("[plugin1] Thread spawned, tid=%p\n", (void*)tid);
    }

private:
    RelativeClock& clock_;
    int            counter_;
};

// ---------- glue into registry ----------
REGISTER_PLUGIN(Plugin1, plugin1);