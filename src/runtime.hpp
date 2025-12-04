#ifndef ILLIXR_RUNTIME_HPP
#define ILLIXR_RUNTIME_HPP

#include <zephyr/kernel.h>
#include <stdio.h>

#include "phonebook_new.hpp"
#include "plugin_registry.hpp"

namespace ILLIXR {

class Runtime {
public:
    explicit Runtime(phonebook_new& pb) : pb_(pb) {}

    // ------------------------------------------------------------------------
    // THE CODE MUST BE HERE (INLINE) BECAUSE THERE IS NO runtime.cpp
    // ------------------------------------------------------------------------

    void initialize(const char* data_path, const char* demo_path) {
        printf("[runtime] Initialize called.\n");
        printf("[runtime]   Data Path: %s\n", data_path ? data_path : "NULL");
        printf("[runtime]   Demo Path: %s\n", demo_path ? demo_path : "NULL");
    }

    void start_all_plugins() {
        printf("[runtime] Starting all plugins...\n");
        
        PluginRegistry& reg = get_plugin_registry();
        
        // Loop through registry using begin()/end()
        // RUntime thread
        printf("[runtime] Thread running runtime: %p\n", k_current_get());
        for (const auto& entry : reg) {
            printf("[runtime] Launching plugin: %s\n", entry.name);
            // This calls the factory function which spawns the plugin's thread
            entry.start_fn(pb_);
        }
        
        printf("[runtime] All plugins launched.\n");
    }

    void shutdown() {
        printf("[runtime] Shutting down...\n");
        // Stop all plugin threads
    }

private:
    phonebook_new& pb_;
};

} // namespace ILLIXR

#endif