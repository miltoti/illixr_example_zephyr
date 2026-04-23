#ifndef ILLIXR_PHONEBOOK_NEW_HPP
#define ILLIXR_PHONEBOOK_NEW_HPP

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <cstdio>

namespace ILLIXR {

class Node;

constexpr size_t MAX_PLUGINS                 = 10;
constexpr size_t MAX_CHANNELS                = 20;
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

    Entry*       begin()       { return entries_; }
    Entry*       end()         { return entries_ + count_; }
    const Entry* begin() const { return entries_; }
    const Entry* end()   const { return entries_ + count_; }
    size_t       size()  const { return count_; }

    // =========================================================================
    // MESSAGING
    // =========================================================================

    struct Subscriber {
        void*  context;
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

    // -------------------------------------------------------------------------
    // publish<MsgT>
    //
    // CRITICAL FIX: We must NOT hold mutex_ while invoking callbacks.
    //
    // Reason: callbacks may themselves call publish() (e.g. openvins publishes
    // PoseMsg and ImuIntegratorInput inside its camera callback). Calling
    // publish() while holding a non-recursive mutex causes an immediate deadlock.
    //
    // Fix: snapshot the matching subscribers under the lock, release the lock,
    // then invoke each callback outside the lock.
    //
    // This is safe because:
    //   • Subscribers are only added (never removed at runtime).
    //   • The snapshot is a plain struct copy — no heap allocation needed.
    //   • Callbacks run with stale-at-worst subscriber data, which is fine
    //     since the subscriber list is stable after plugin construction.
    // -------------------------------------------------------------------------
    template<typename MsgT>
    void publish(const char* sender,
                 const char* receiver,
                 const MsgT& msg)
    {
        // --- 1. Snapshot subscribers under lock ------------------------------
        Subscriber snapshot[MAX_SUBSCRIBERS_PER_CHANNEL];
        size_t     snap_count = 0;
        uintptr_t  tid        = type_id<MsgT>();

        k_mutex_lock(&mutex_, K_FOREVER);
        Channel* ch = find_channel(sender, receiver);
        if (ch) {
            for (size_t i = 0; i < ch->sub_count; i++) {
                if (ch->subs[i].type_id == tid) {
                    snapshot[snap_count++] = ch->subs[i];
                }
            }
        }
        k_mutex_unlock(&mutex_);   // ← released BEFORE any callback fires

        // --- 2. Invoke callbacks outside lock --------------------------------
        for (size_t i = 0; i < snap_count; i++) {
            auto cb = reinterpret_cast<void (*)(void*, const MsgT&)>(
                          snapshot[i].callback);
            cb(snapshot[i].context, msg);
        }
    }

private:
    k_mutex mutex_;
    Entry   entries_[MAX_PLUGINS];
    size_t  count_;

    Channel channels_[MAX_CHANNELS];
    size_t  channel_count_;

    Channel* find_channel(const char* sender, const char* receiver) {
        for (size_t i = 0; i < channel_count_; i++) {
            if (!strcmp(channels_[i].sender,   sender) &&
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

        Channel& new_ch    = channels_[channel_count_++];
        new_ch.sender      = sender;
        new_ch.receiver    = receiver;
        new_ch.sub_count   = 0;
        return &new_ch;
    }
};

phonebook_new& get_phonebook();
void           init_phonebook_global();

} // namespace ILLIXR

#endif // ILLIXR_PHONEBOOK_NEW_HPP