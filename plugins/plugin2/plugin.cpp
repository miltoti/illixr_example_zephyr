#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "../../src/node.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"

using namespace ILLIXR;

// -------------------------
// Message types
// -------------------------
constexpr size_t MAX_TIMESTAMPS = 16;

struct SensorMsg {
    int timestamp;
};

struct SummaryMsg {
    int timestamps[MAX_TIMESTAMPS];
    size_t count;
};

// -------------------------
// Plugin2 (contains Node)
// -------------------------
class Plugin2 {
public:
    Plugin2(phonebook_new& pb)
        : node_{}, received_count_{0}
    {
        printk("[plugin2] Constructor\n");
        node_.initialize(pb, "plugin2");
    }

    void configure() {
        printk("[plugin2] configuring subscriptions\n");

        node_.subscribe_from<SensorMsg>(
            "plugin1",
            &Plugin2::on_sensor,
            this
        );
    }

    void start() {
        printk("[plugin2] start() entered\n");
        while (true) {
            printk("[plugin2] alive\n");
            k_sleep(K_SECONDS(1));
        }
    }

private:
    Node node_;     // <<--- Contains Node
    int received_[MAX_TIMESTAMPS];
    size_t received_count_;

    static void on_sensor(void* ctx, const SensorMsg& msg) {
        auto* self = static_cast<Plugin2*>(ctx);

        printk("[plugin2] received timestamp=%d\n", msg.timestamp);

        // Store timestamp
        if (self->received_count_ < MAX_TIMESTAMPS) {
            self->received_[self->received_count_++] = msg.timestamp;
        }

        // After 6 timestamps → pause 3 sec → send summary
        if (self->received_count_ == 6) {
            printk("[plugin2] collected 6 → waiting 3s\n");
            k_sleep(K_SECONDS(3));

            SummaryMsg summary{};
            summary.count = self->received_count_;

            for (size_t i = 0; i < summary.count; i++) {
                summary.timestamps[i] = self->received_[i];
            }

            printk("[plugin2] sending SummaryMsg to p3 (count=%zu)\n", summary.count);
            self->node_.publish_to<SummaryMsg>("p3", summary);

            self->received_count_ = 0;
        }
    }
};

// -------------------------
// Factory
// -------------------------
void start_plugin2(phonebook_new& pb) {
    printk("[plugin2] start_plugin2() called\n");
    static Plugin2 instance{pb};
    instance.configure();
}

REGISTER_PLUGIN(plugin2);
