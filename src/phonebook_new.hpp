#ifndef ILLIXR_PHONEBOOK_NEW_HPP
#define ILLIXR_PHONEBOOK_NEW_HPP

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace ILLIXR {

class Node;

constexpr size_t MAX_PLUGINS = 10;
constexpr size_t MAX_CHANNELS = 20;
constexpr size_t MAX_SUBSCRIBERS_PER_CHANNEL = 4;

class phonebook_new {
public:
    struct Entry {
        const char* name;
        Node*       instance;
    };

    phonebook_new() : count_(0), channel_count_(0) {
        k_mutex_init(&mutex_);
    }

    bool register_plugin(const char* name, Node* instance) {
        printf("Registering plugin: %s\n", name);
        k_mutex_lock(&mutex_, K_FOREVER);
        if (count_ >= MAX_PLUGINS) {
            k_mutex_unlock(&mutex_);
            return false;
        }
        entries_[count_++] = {name, instance};
        k_mutex_unlock(&mutex_);
        return true;
    }

    Node* lookup(const char* name) {
        k_mutex_lock(&mutex_, K_FOREVER);
        for (size_t i = 0; i < count_; i++) {
            if (strcmp(name, entries_[i].name) == 0) {
                Node* node = entries_[i].instance;
                k_mutex_unlock(&mutex_);
                return node;
            }
        }
        k_mutex_unlock(&mutex_);
        return nullptr;
    }

    Entry* begin()       { return entries_; }
    Entry* end()         { return entries_ + count_; }
    const Entry* begin() const { return entries_; }
    const Entry* end()   const { return entries_ + count_; }
    size_t size() const  { return count_; }

    // ================================================================
    // MESSAGING
    // ================================================================

    struct Subscriber {
        void* context;
        void (*callback)(void*, const void*);
        uintptr_t type_id;
    };

    struct Channel {
        const char* sender;
        const char* receiver;
        Subscriber  subs[MAX_SUBSCRIBERS_PER_CHANNEL];
        size_t      sub_count = 0;
    };

    template<typename MsgT>
    static uintptr_t type_id() {
        static char id;
        return (uintptr_t)&id;
    }

    template<typename MsgT>
    void subscribe(const char* sender,
                   const char* receiver,
                   void (*cb)(void*, const MsgT&),
                   void* ctx)
    {
        k_mutex_lock(&mutex_, K_FOREVER);
        Channel* ch = find_or_create_channel(sender, receiver);
        if (!ch || ch->sub_count >= MAX_SUBSCRIBERS_PER_CHANNEL) {
            k_mutex_unlock(&mutex_);
            return;
        }

        Subscriber& s = ch->subs[ch->sub_count++];
        s.context  = ctx;
        s.callback = reinterpret_cast<void (*)(void*, const void*)>(cb);
        s.type_id  = type_id<MsgT>();
        k_mutex_unlock(&mutex_);
    }

    template<typename MsgT>
    void publish(const char* sender,
                 const char* receiver,
                 const MsgT& msg)
    {
        k_mutex_lock(&mutex_, K_FOREVER);
        Channel* ch = find_channel(sender, receiver);
        if (ch) {
            for (size_t i = 0; i < ch->sub_count; i++) {
                if (ch->subs[i].type_id == type_id<MsgT>()) {
                    auto cb = reinterpret_cast<void (*)(void*, const MsgT&)>(ch->subs[i].callback);
                    cb(ch->subs[i].context, msg);
                }
            }
        }
        k_mutex_unlock(&mutex_);
    }

    // ================================================================
    // PERIODIC PUBLISHING
    // ================================================================
    template<typename MsgT, typename Generator>
    void publish_periodic(const char* sender,
                          const char* receiver,
                          uint32_t interval_ms,
                          Generator gen)
    {
        struct Args {
            phonebook_new* self;
            const char* sender;
            const char* receiver;
            Generator gen;
            uint32_t interval;
        };

        Args* args = new Args{this, sender, receiver, gen, interval_ms};

        auto thread_fn = [](void* a, void*, void*) {
            Args* arg = static_cast<Args*>(a);
            while (1) {
                MsgT msg = arg->gen();
                arg->self->template publish<MsgT>(arg->sender, arg->receiver, msg);
                k_msleep(arg->interval);
            }
        };

        // Each plugin can have multiple periodic publishers
        static K_THREAD_STACK_DEFINE(p_stack, 1024);
        static struct k_thread p_thread;

        k_thread_create(
            &p_thread,
            p_stack,
            K_THREAD_STACK_SIZEOF(p_stack),
            thread_fn,
            args, nullptr, nullptr,
            K_PRIO_PREEMPT(5),
            0,
            K_NO_WAIT
        );
    }

private:
    k_mutex mutex_;
    Entry entries_[MAX_PLUGINS];
    size_t count_;

    Channel channels_[MAX_CHANNELS];
    size_t  channel_count_;

    Channel* find_channel(const char* sender, const char* receiver) {
        for (size_t i = 0; i < channel_count_; i++) {
            if (!strcmp(channels_[i].sender, sender) &&
                !strcmp(channels_[i].receiver, receiver)) {
                return &channels_[i];
            }
        }
        return nullptr;
    }

    Channel* find_or_create_channel(const char* sender, const char* receiver) {
        Channel* ch = find_channel(sender, receiver);
        if (ch) return ch;
        if (channel_count_ >= MAX_CHANNELS) return nullptr;

        Channel& new_ch = channels_[channel_count_++];
        new_ch.sender    = sender;
        new_ch.receiver  = receiver;
        new_ch.sub_count = 0;
        return &new_ch;
    }
};

// GLOBAL ACCESS POINT
phonebook_new& get_phonebook();
void init_phonebook_global();

} // namespace ILLIXR

#endif
