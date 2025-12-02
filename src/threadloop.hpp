// threadloop.hpp — NO thread creation inside. Runtime owns the thread.

#pragma once

#include "plugin.hpp"
#include <zephyr/kernel.h>
#include <atomic>

namespace ILLIXR {

class threadloop : public Plugin {
public:
    threadloop(phonebook_new& pb, const char* name)
        : Plugin{pb, name} { }

    /**
     * Runtime will call this inside the Zephyr thread it created.
     * This is our main loop.
     */
    void start() override {
        _p_thread_setup();

        while (!should_terminate()) {
            skip_option s = _p_should_skip();

            switch (s) {
            case skip_option::skip_and_yield:
                k_yield();
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
                internal_stop();
                break;
            }
        }
    }

    void internal_stop() {
        stop_flag_.store(true);
    }

protected:
    enum class skip_option {
        run,
        skip_and_spin,
        skip_and_yield,
        stop,
    };

    virtual skip_option _p_should_skip()     { return skip_option::run; }
    virtual void        _p_thread_setup()    { }
    virtual void        _p_one_iteration()   = 0;

    bool should_terminate() const {
        return stop_flag_.load();
    }

protected:
    std::size_t iteration_no = 0;
    std::size_t skip_no      = 0;

private:
    std::atomic<bool> stop_flag_{false};
};

} // namespace ILLIXR
