#ifndef ILLIXR_NODE_HPP
#define ILLIXR_NODE_HPP

#include "phonebook_new.hpp"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace ILLIXR {

constexpr size_t MAX_PLUGIN_NAME_LEN = 32;

/**
 * RTOS-safe Node
 *
 * IMPORTANT:
 *   - Constructor does NOT touch the phonebook.
 *   - Node is registered ONLY after Zephyr kernel boot,
 *     via an explicit call to initialize().
 *
 * This prevents global static constructors from crashing Zephyr.
 */
class Node {
public:
    typedef void (*MsgCallbackFn)(void* context, const void* msg);

    /** Constructor: does NOT register with phonebook yet */
    Node()
        : pb_(nullptr) {
        name_[0] = '\0';
    }

    /** Constructor with name only (still does NOT register) */
    explicit Node(const char* name)
        : pb_(nullptr) {
        strncpy(name_, name, MAX_PLUGIN_NAME_LEN - 1);
        name_[MAX_PLUGIN_NAME_LEN - 1] = '\0';
    }

    /** Explicit initialization; must be called after Zephyr boot */
    void initialize(phonebook_new& pb, const char* name) {
        pb_ = &pb;

        strncpy(name_, name, MAX_PLUGIN_NAME_LEN - 1);
        name_[MAX_PLUGIN_NAME_LEN - 1] = '\0';

        // Safe: phonebook now exists, Zephyr kernel is alive
        pb_->register_plugin(name_, this);
    }

    virtual ~Node() = default;

    /**
     * Called inside thread context after all plugins are registered.
     * Plugins override this to start their logic.
     */
    virtual void start() {}

    // -----------------------------
    // Messaging utilities
    // -----------------------------
    template<typename MsgT>
    void subscribe_from(const char* sender_name,
                        void (*callback)(void* ctx, const MsgT&),
                        void* context) {
        if (!pb_) { return; }
        pb_->subscribe<MsgT>(sender_name, name_, callback, context);
    }

    template<typename MsgT>
    void publish_to(const char* receiver_name, const MsgT& msg) {
        if (!pb_) { return; }
        pb_->publish<MsgT>(name_, receiver_name, msg);
    }

    template<typename MsgT, typename MsgGenerator>
    void publish_to_periodic(const char* receiver_name,
                             uint32_t interval_ms,
                             MsgGenerator generator) {
        if (!pb_) { return; }
        pb_->publish_periodic<MsgT>(name_, receiver_name, interval_ms, generator);
    }

    const char* name() const { return name_; }

private:
    phonebook_new* pb_;                 // safe: only set after init
    char           name_[MAX_PLUGIN_NAME_LEN];
};

} // namespace ILLIXR

#endif // ILLIXR_NODE_HPP
