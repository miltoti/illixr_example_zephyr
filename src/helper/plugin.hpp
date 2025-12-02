// plugin.hpp
#pragma once

#include <zephyr/kernel.h>
#include <atomic>
#include <cassert>

namespace ILLIXR {

/**
 * Base class for threaded plugins.
 *
 * - NOT a Node.
 * - Owns a Zephyr thread and stack.
 * - start()/stop() manage the thread.
 * - Calls plugin_thread_main() in the worker thread.
 *
 * Each plugin is free to create/register Node objects for communication.
 */
class Plugin {
public:
    Plugin() = default;
    explicit Plugin(const char* name) : name_{name} { }

    virtual ~Plugin() {
        internal_stop();
        // Optional: join if enabled in Zephyr config
    }

    const char* name() const { return name_; }

    /** Start the plugin's thread. */
    virtual void start() {
        if (thread_started_) {
            return;
        }

        _stop_flag.store(false);
        thread_started_ = true;

        _tid = k_thread_create(
            &_thread,
            _stack,
            K_THREAD_STACK_SIZEOF(_stack),
            &Plugin::thread_entry_trampoline,
            this, nullptr, nullptr,
            K_PRIO_PREEMPT(PRIORITY),
            0,
            K_NO_WAIT
        );

        if (_tid != nullptr && name_) {
            k_thread_name_set(_tid, name_);
        }

        assert(_tid != nullptr);
    }

    /** Request the thread to stop and optionally join. */
    virtual void stop() {
        internal_stop();

    #if defined(CONFIG_THREAD_MONITOR)
        if (thread_started_ && _tid != nullptr) {
            k_thread_join(_tid, K_FOREVER);
        }
    #endif

        thread_started_ = false;
    }

    /** Ask the plugin's thread to terminate. */
    virtual void internal_stop() {
        _stop_flag.store(true);
    }

    bool should_terminate() const {
        return _stop_flag.load();
    }

protected:
    /** Implement this in subclasses; runs in the worker thread. */
    virtual void plugin_thread_main() = 0;

private:
    static void thread_entry_trampoline(void* p1, void* p2, void* p3) {
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        auto* self = static_cast<Plugin*>(p1);
        if (!self) return;
        self->plugin_thread_main();
        self->thread_started_ = false;
    }

    const char*       name_{nullptr};
    std::atomic<bool> _stop_flag{false};

    struct k_thread _thread{};
    k_tid_t          _tid{nullptr};
    bool             thread_started_{false};

    static constexpr size_t STACK_SIZE = 2048;
    static constexpr int    PRIORITY   = 5;
    K_THREAD_STACK_MEMBER(_stack, STACK_SIZE);
};

} // namespace ILLIXR
