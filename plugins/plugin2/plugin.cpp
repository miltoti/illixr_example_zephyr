// plugins/plugin2/plugin.cpp

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "../../src/plugin.hpp"
#include "../../src/plugin_registry.hpp"

using namespace ILLIXR;

// ---------- static Zephyr thread objects for this plugin ----------
static K_THREAD_STACK_DEFINE(plugin2_stack, 4096);
static struct k_thread       plugin2_thread;

constexpr size_t MAX_TIMESTAMPS = 16;

// Same as in plugin1
struct SensorMsg {
    int64_t t_ns;
};

struct SummaryMsg {
    int64_t timestamps[MAX_TIMESTAMPS];
    size_t  count;
};

class Plugin2 : public Plugin {
public:
    explicit Plugin2(phonebook_new& pb)
        : Plugin{pb, "plugin2"}
        , received_count_{0}
    {
        printk("[plugin2] ctor\n");
        configure();
    }

    void configure() {
        printk("[plugin2] configuring subscription from plugin1\n");

        node().subscribe_from<SensorMsg>(
            "plugin1",
            &Plugin2::on_sensor,
            this
        );
    }

    // --------- plugin worker loop (runs in plugin2 thread) ---------
    void thread_loop() {
        printk("[plugin2] thread_loop start\n");

        while (true) {
            // If plugin2 ever uses publish_to_periodic, this will drive it.
            node().service_periodic();

            // any other background work for plugin2 can go here

            k_msleep(10);
        }
    }

    static void thread_entry(void* p1, void*, void*) {
        auto* self = static_cast<Plugin2*>(p1);
        self->thread_loop();
    }

    void start() override {
        printk("[plugin2] Spawning thread...\n");

        k_tid_t tid = k_thread_create(
            &plugin2_thread,
            plugin2_stack,
            K_THREAD_STACK_SIZEOF(plugin2_stack),
            &Plugin2::thread_entry,
            this, nullptr, nullptr,
            5,  // priority
            0,
            K_NO_WAIT
        );

        k_thread_name_set(tid, "plugin2");
        printk("[plugin2] Thread spawned, tid=%p\n", (void*)tid);
    }

private:
    int64_t received_[MAX_TIMESTAMPS];
    size_t  received_count_;

    // Subscription callback (runs in publisher’s thread, i.e. plugin1’s)
    static void on_sensor(void* ctx, const SensorMsg& msg) {
        auto* self = static_cast<Plugin2*>(ctx);

        printk("[plugin2] received t_ns=%lld\n", (long long)msg.t_ns);

        if (self->received_count_ < MAX_TIMESTAMPS) {
            self->received_[self->received_count_++] = msg.t_ns;
        }

        if (self->received_count_ == 6) {
            printk("[plugin2] collected 6 → waiting 3s\n");
            k_sleep(K_SECONDS(3));

            SummaryMsg summary{};
            summary.count = self->received_count_;

            for (size_t i = 0; i < summary.count; ++i) {
                summary.timestamps[i] = self->received_[i];
            }

            printk("[plugin2] sending SummaryMsg to p3 (count=%zu)\n", summary.count);
            self->node().publish_to<SummaryMsg>("p3", summary);

            self->received_count_ = 0;
        }
    }
};

// ---------- glue into registry ----------
REGISTER_PLUGIN(Plugin2, plugin2);