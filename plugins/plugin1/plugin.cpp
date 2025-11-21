#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "../../src/node.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"

using namespace ILLIXR;

// -------------------------
// Message type
// -------------------------
struct SensorMsg {
    int timestamp;
};

// -------------------------
// Plugin1 (contains Node)
// -------------------------
class Plugin1 {
public:
    Plugin1(phonebook_new& pb)
        : node_{}, counter_{0}
    {
        printk("[plugin1] Constructor\n");
        node_.initialize(pb, "plugin1");
    }

    void configure() {
        printk("[plugin1] configuring: setting up periodic publisher\n");

        node_.publish_to_periodic<SensorMsg>(
            "plugin2",
            500,
            [this]() -> SensorMsg {
                printk("[plugin1] sending timestamp=%d\n", counter_);
                return SensorMsg{ counter_++ };
            }
        );
    }

    void start() {
        printk("[plugin1] start() entered\n");
        while (true) {
            printk("[plugin1] alive\n");
            k_sleep(K_SECONDS(1));
        }
    }

private:
    Node node_;   // <<--- Contains a Node
    int counter_;
};

// -------------------------
// Factory
// -------------------------
void start_plugin1(phonebook_new& pb) {
    printk("[plugin1] start_plugin1() called\n");
    static Plugin1 instance{pb};
    instance.configure();
}

REGISTER_PLUGIN(plugin1);
