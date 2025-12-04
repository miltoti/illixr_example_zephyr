#ifndef ILLIXR_NODE_HPP
#define ILLIXR_NODE_HPP

#include "phonebook_new.hpp"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <vector>
#include <memory>
#include <zephyr/kernel.h> // for k_uptime_get

namespace ILLIXR {

constexpr size_t MAX_PLUGIN_NAME_LEN = 32;

/**
 * RTOS-safe Node
 *
 * IMPORTANT:
 * - Registration is handled by the Plugin wrapper.
 * - Periodic publishing does NOT create extra threads.
 * Plugins must call Node::service_periodic() from their own thread loop.
 */
class Node {
public:
    typedef void (*MsgCallbackFn)(void* context, const void* msg);

    Node() : pb_{nullptr} {
        name_[0] = '\0';
    }

    explicit Node(const char* name) : pb_{nullptr} {
        strncpy(name_, name, MAX_PLUGIN_NAME_LEN - 1);
        name_[MAX_PLUGIN_NAME_LEN - 1] = '\0';
    }

    virtual ~Node() = default;

    void initialize(phonebook_new& pb, const char* name);
    void shutdown();

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

    /**
     * Registers a periodic job.
     * DOES NOT create a thread. The plugin's thread must call service_periodic().
     */
    template<typename MsgT, typename MsgGenerator>
    void publish_to_periodic(const char* receiver_name,
                             uint32_t interval_ms,
                             MsgGenerator generator) {
        if (!pb_) { return; }

        using JobT = PeriodicJob<MsgT, MsgGenerator>;
        auto job   = std::make_unique<JobT>(
            pb_,
            name_,
            receiver_name,
            interval_ms,
            generator
        );
        periodic_jobs_.push_back(std::move(job));
    }

    /**
     * Drives the periodic jobs. Call this in your plugin's while(1) loop.
     */
    void service_periodic();

    const char* name() const { return name_; }

private:
    // Base class for polymorphic storage of templated jobs
    struct PeriodicJobBase {
        virtual ~PeriodicJobBase() = default;
        virtual void tick(uint64_t now_ms) = 0;
    };

    template<typename MsgT, typename Generator>
    struct PeriodicJob : PeriodicJobBase {
        phonebook_new* pb;
        char           sender[MAX_PLUGIN_NAME_LEN];
        char           receiver[MAX_PLUGIN_NAME_LEN];
        Generator      gen;
        uint32_t       interval_ms;
        uint64_t       next_fire_ms;

        PeriodicJob(phonebook_new* pb_,
                    const char* sender_name,
                    const char* receiver_name,
                    uint32_t interval,
                    Generator g)
            : pb{pb_}
            , gen{g}
            , interval_ms{interval}
            , next_fire_ms{k_uptime_get() + interval}
        {
            strncpy(sender,   sender_name,   MAX_PLUGIN_NAME_LEN - 1);
            sender[MAX_PLUGIN_NAME_LEN - 1] = '\0';

            strncpy(receiver, receiver_name, MAX_PLUGIN_NAME_LEN - 1);
            receiver[MAX_PLUGIN_NAME_LEN - 1] = '\0';
        }

        void tick(uint64_t now_ms) override {
            if (!pb) { return; }
            if (now_ms >= next_fire_ms) {
                MsgT msg = gen();
                pb->template publish<MsgT>(sender, receiver, msg);
                next_fire_ms = now_ms + interval_ms;
            }
        }
    };

    phonebook_new* pb_;
    char           name_[MAX_PLUGIN_NAME_LEN];
    std::vector<std::unique_ptr<PeriodicJobBase>> periodic_jobs_;
};

} // namespace ILLIXR

#endif // ILLIXR_NODE_HPP