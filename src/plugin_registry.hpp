#ifndef ILLIXR_PLUGIN_REGISTRY_HPP
#define ILLIXR_PLUGIN_REGISTRY_HPP

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "phonebook_new.hpp"

namespace ILLIXR {

using plugin_start_fn_t = void (*)(phonebook_new&);

constexpr size_t MAX_REGISTERED_PLUGINS = 20;

struct plugin_registry_entry {
    const char*        name;
    plugin_start_fn_t  start_fn;
};

class PluginRegistry {
public:
    PluginRegistry() : count_{0} {
        printf("[registry] PluginRegistry constructed @%p\n", (void*)this);
    }

    bool register_plugin(const char* name, plugin_start_fn_t fn) {
        printf("[registry] Registering plugin '%s' (current count=%zu)\n",
               name, count_);

        if (count_ >= MAX_REGISTERED_PLUGINS) {
            printf("[registry] ERROR: MAX_REGISTERED_PLUGINS reached!\n");
            return false;
        }

        entries_[count_].name     = name;
        entries_[count_].start_fn = fn;
        ++count_;

        printf("[registry] DONE registering '%s' → new count=%zu\n",
               name, count_);
        return true;
    }

    size_t size() const { return count_; }

    const plugin_registry_entry* begin() const { return entries_; }
    const plugin_registry_entry* end()   const { return entries_ + count_; }

private:
    plugin_registry_entry entries_[MAX_REGISTERED_PLUGINS];
    size_t count_;
};

inline PluginRegistry& get_plugin_registry() {
    printf("[registry] get_plugin_registry() called\n");
    static PluginRegistry registry;
    return registry;
}

// ===========================================================================
// UPDATED REGISTER_PLUGIN macro with HEAVY DEBUGGING
// Shows:
//   • file + line where macro expands
//   • static constructor address
//   • proof the static object exists (prints its address)
//   • proof the ctor runs BEFORE runtime
// ===========================================================================
#define REGISTER_PLUGIN(name)                                                      \
    extern void start_##name(::ILLIXR::phonebook_new& pb);                         \
    namespace {                                                                    \
    struct plugin_registrar_##name {                                               \
        plugin_registrar_##name() {                                                \
            printf("[registry] STATIC CTOR FIRED for %s\n", #name);                \
            printf("[registry]   from file: %s:%d\n", __FILE__, __LINE__);         \
            printf("[registry]   ctor this=%p\n", (void*)this);                    \
            ::ILLIXR::get_plugin_registry().register_plugin(                       \
                #name, &start_##name);                                             \
        }                                                                          \
    };                                                                             \
    static plugin_registrar_##name registrar_instance_##name                       \
        __attribute__((used, retain));                                             \
    }

} // namespace ILLIXR

#endif // ILLIXR_PLUGIN_REGISTRY_HPP
