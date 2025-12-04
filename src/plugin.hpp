#pragma once

#include "node.hpp"
#include "phonebook_new.hpp"
#include <zephyr/kernel.h>

namespace ILLIXR {

/**
 * Active Object Plugin base:
 * - Owns a Node.
 * - Derived class MUST create its own Zephyr thread in start().
 */
class Plugin {
public:
    Plugin(phonebook_new& pb, const char* name)
        : node_{}
        , should_stop_{false} {
        // 1. Initialize Node's internal state
        node_.initialize(pb, name);

        // 2. Register the Node instance with the phonebook
        pb.register_plugin(name, &node_);
    }

    virtual ~Plugin() = default;

    /**
     * Start the plugin worker thread.
     * MUST return immediately (non-blocking).
     * Implementation should call k_thread_create using a stack defined in the derived class.
     */
    virtual void start() = 0;

    /**
     * Stop the plugin worker thread (optional).
     */
    virtual void stop() {
        should_stop_ = true;
    }

    // Access to the underlying Node
    Node&       node()       { return node_; }
    const Node& node() const { return node_; }

protected:
    Node node_;
    volatile bool should_stop_;

    /**
     * Helper for the threaded loop.
     * The derived class's thread entry point should call this.
     *
     * @param sleep_ms - The cycle tick rate (prevents CPU starvation).
     */
    void run_loop(uint32_t sleep_ms = 10) {
        while (!should_stop_) {
            // 1. Check and fire any periodic jobs registered in the Node
            node_.service_periodic();

            // 2. Yield/Sleep to allow other threads to run
            k_msleep(sleep_ms);
        }
    }
};

} // namespace ILLIXR