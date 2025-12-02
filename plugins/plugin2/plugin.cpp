// plugins/plugin2/plugin.cpp
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "../../src/plugin.hpp"
#include "../../src/plugin_registry.hpp"

using namespace ILLIXR;

constexpr size_t MAX_TIMESTAMPS = 16;

struct SensorMsg {
    int timestamp;
};

struct SummaryMsg {
    int timestamps[MAX_TIMESTAMPS];
    size_t count;
};

class Plugin2 : public Plugin {
public:
    explicit Plugin2(phonebook_new& pb)
        : Plugin{pb, "plugin2"}
        , received_count_{0}
    {
        printk("[plugin2] Constructor\n");
    }

    void configure() {
        printk("[plugin2] configuring subscriptions\n");

        node().subscribe_from<SensorMsg>(
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
    int    received_[MAX_TIMESTAMPS];
    size_t received_count_;

    static void on_sensor(void* ctx, const SensorMsg& msg) {
        auto* self = static_cast<Plugin2*>(ctx);

        printk("[plugin2] received timestamp=%d\n", msg.timestamp);

        if (self->received_count_ < MAX_TIMESTAMPS) {
            self->received_[self->received_count_++] = msg.timestamp;
        }

        if (self->received_count_ == 6) {
            printk("[plugin2] collected 6 → waiting 3s\n");
            k_sleep(K_SECONDS(3));

            SummaryMsg summary{};
            summary.count = self->received_count_;

            for (size_t i = 0; i < summary.count; i++) {
                summary.timestamps[i] = self->received_[i];
            }

            printk("[plugin2] sending SummaryMsg to p3 (count=%zu)\n", summary.count);
            self->node().publish_to<SummaryMsg>("p3", summary);

            self->received_count_ = 0;
        }
    }
};

void start_plugin2(phonebook_new& pb) {
    printk("[plugin2] start_plugin2() called\n");
    static Plugin2 instance{pb};
    instance.configure();      // NO instance.start()
}

REGISTER_PLUGIN(plugin2);
