// threadloop.hpp
//
// Base class for ILLIXR plugins that own a single worker thread.
//
// WHY NO TEMPLATE:
//   Zephyr's K_THREAD_STACK_DEFINE / K_THREAD_STACK_MEMBER use special
//   linker-section attributes that cannot be instantiated inside a template
//   class. The stack must be a concrete static symbol at file scope.
//
// PATTERN:
//   Each derived plugin defines its stack at file scope:
//
//     K_THREAD_STACK_DEFINE(my_plugin_stack, 65536);
//
//   Then passes it to threadloop via the constructor:
//
//     class MyPlugin : public threadloop {
//     public:
//         MyPlugin(phonebook_new& pb)
//             : threadloop{pb, "my_plugin",
//                          my_plugin_stack,
//                          K_THREAD_STACK_SIZEOF(my_plugin_stack),
//                          5}
//         { ... }
//     };
//
//   threadloop::start() calls k_thread_create with the provided stack.
//   The thread calls _p_thread_setup() once, then signals stoplight_ready,
//   then loops calling _p_should_skip() / _p_one_iteration() until stopped.

#pragma once

#include "plugin.hpp"
#include "stoplight.hpp"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <cstdio>

namespace ILLIXR {

class threadloop : public Plugin {
public:
    threadloop(phonebook_new&  pb,
               const char*     name,
               k_thread_stack_t* stack,
               size_t          stack_size,
               int             priority = 5)
        : Plugin{pb, name}
        , stack_{stack}
        , stack_size_{stack_size}
        , priority_{priority}
        , tid_{nullptr}
    {
        atomic_set(&stop_flag_, 0);
    }

    void start() override {
        printf("[threadloop:%s] start() stack_size=%zu priority=%d\n",
               node_.name(), stack_size_, priority_);

        atomic_set(&stop_flag_, 0);

        tid_ = k_thread_create(
            &thread_,
            stack_,
            stack_size_,
            thread_entry,
            this, nullptr, nullptr,
            K_PRIO_PREEMPT(priority_),
            0,
            K_NO_WAIT
        );

        if (tid_) {
            k_thread_name_set(tid_, node_.name());
            printf("[threadloop:%s] thread spawned tid=%p\n",
                   node_.name(), (void*)tid_);
        } else {
            printf("[threadloop:%s] ERROR: k_thread_create returned null!\n",
                   node_.name());
        }
    }

    void stop() override {
        printf("[threadloop:%s] stop() called\n", node_.name());
        atomic_set(&stop_flag_, 1);
    }

    bool should_terminate() const {
        return atomic_get(&stop_flag_) != 0;
    }

protected:
    enum class skip_option {
        run,
        skip_and_spin,
        skip_and_yield,
        stop,
    };

    virtual void        _p_thread_setup()  { }
    virtual skip_option _p_should_skip()   { return skip_option::run; }
    virtual void        _p_one_iteration() = 0;

    size_t iteration_no = 0;
    size_t skip_no      = 0;

private:
    void run() {
        printf("[threadloop:%s] worker running tid=%p  stop_flag_=%ld\n",
               node_.name(), k_current_get(), (long)atomic_get(&stop_flag_));

        _p_thread_setup();

        // ── Signal runtime that this thread is fully set up ───────────────
        // All subscriptions are registered at this point.
        // Runtime is waiting in start_all_plugins() for reg.size() signals
        // before allowing data to flow.
        printf("[threadloop:%s] setup done — signalling stoplight_ready\n",
               node_.name());
        k_sem_give(&stoplight_ready);
        // ─────────────────────────────────────────────────────────────────

        printf("[threadloop:%s] entering loop  stop_flag_=%ld\n",
               node_.name(), (long)atomic_get(&stop_flag_));

        while (!should_terminate()) {
            switch (_p_should_skip()) {
            case skip_option::skip_and_yield:
                k_msleep(1);
                ++skip_no;
                break;

            case skip_option::skip_and_spin:
                ++skip_no;
                break;

            case skip_option::run:
                _p_one_iteration();
                ++iteration_no;
                skip_no = 0;
                break;

            case skip_option::stop:
                printf("[threadloop:%s] _p_should_skip() → stop  "
                       "iter=%zu skip=%zu\n",
                       node_.name(), iteration_no, skip_no);
                atomic_set(&stop_flag_, 1);
                break;
            }
        }

        printf("[threadloop:%s] loop exited  iter=%zu skip=%zu\n",
               node_.name(), iteration_no, skip_no);
    }

    static void thread_entry(void* p1, void*, void*) {
        static_cast<threadloop*>(p1)->run();
    }

    k_thread_stack_t* stack_;
    size_t            stack_size_;
    int               priority_;
    atomic_t          stop_flag_;
    struct k_thread   thread_;
    k_tid_t           tid_;
};

} // namespace ILLIXR