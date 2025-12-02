// plugins/plugin1/plugin.cpp

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "../../src/node.hpp"
#include "../../src/phonebook_new.hpp"
#include "../../src/plugin_registry.hpp"
#include "../../src/relative_clock.hpp"

using namespace ILLIXR;

// ---------------------------------------------------------
// Message type
// ---------------------------------------------------------
struct SensorMsg {
    std::int64_t t_ns;  // timestamp in nanoseconds (relative to global RelativeClock start)
};

// ---------------------------------------------------------
// Plugin1 (contains Node)
// ---------------------------------------------------------
class Plugin1 {
public:
    explicit Plugin1(phonebook_new& pb)
        : node_{}
        , clock_{get_global_relative_clock()}
        , counter_{0}
    {
        printk("[plugin1] Constructor\n");
        node_.initialize(pb, "plugin1");
    }

    void configure() {
        printk("[plugin1] configuring: setting up periodic publisher\n");

        // Publish a SensorMsg every 500 ms to "plugin2"
        node_.publish_to_periodic<SensorMsg>(
            "plugin2",
            500,   // period in ms
            [this]() -> SensorMsg {
                // Use global RelativeClock for timestamp
                std::int64_t t_ns = clock_.now_ns();

                printk("[plugin1] sending sample #%d at t=%lld ns\n",
                       counter_, (long long)t_ns);

                ++counter_;
                return SensorMsg{ t_ns };
            }
        );
    }

    // Optional: not used by runtime (runtime threads over Node and calls Node::start())
    void start() {
        printk("[plugin1] start() entered\n");
        while (true) {
            printk("[plugin1] alive\n");
            k_sleep(K_SECONDS(1));
        }
    }

private:
    Node           node_;   // Node used for pub/sub + registration in phonebook
    RelativeClock& clock_;  // Reference to global RelativeClock
    int            counter_;
};

// ---------------------------------------------------------
// Factory
// ---------------------------------------------------------
void start_plugin1(phonebook_new& pb) {
    printk("[plugin1] start_plugin1() called\n");
    static Plugin1 instance{pb};
    instance.configure();   // DO NOT call instance.start(); runtime owns threads via Node
}

// Keep existing registry macro as-is
REGISTER_PLUGIN(plugin1);
