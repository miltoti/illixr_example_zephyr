#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "../../src/node.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"

using namespace ILLIXR;

// -------------------------
// Message type
// -------------------------
constexpr size_t MAX_TIMESTAMPS = 16;

struct SummaryMsg {
    int timestamps[MAX_TIMESTAMPS];
    size_t count;
};

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

        node_.subscribe_from<SummaryMsg>(
            "plugin2",
            &P3::on_summary,
            this
        );
    }

    void start() {
        printk("[p3] start() thread running\n");
        while (true) {
            printk("[p3] alive\n");
            k_sleep(K_SECONDS(1));
        }
    }

private:
    Node node_;  // <<--- Contains Node

    static void on_summary(void* ctx, const SummaryMsg& msg) {
        auto* self = static_cast<P3*>(ctx);

        printk("[p3] received SummaryMsg (count=%zu)\n", msg.count);

        for (size_t i = 0; i < msg.count; i++) {
            printk("[p3]   timestamps[%zu] = %d\n", i, msg.timestamps[i]);
        }
    }
};

// -------------------------
// Factory
// -------------------------
void start_p3(phonebook_new& pb) {
    printk("[p3] start_p3() called\n");
    static P3 instance{pb};
    instance.configure();
}

REGISTER_PLUGIN(p3);
