/*#ifndef ILLIXR_RUNTIME_HPP
#define ILLIXR_RUNTIME_HPP

#include <zephyr/kernel.h>
#include <stdio.h>

#include "phonebook_new.hpp"
#include "plugin_registry.hpp"

namespace ILLIXR {

class Runtime {
public:
    explicit Runtime(phonebook_new& pb) : pb_(pb) {}

    /**
     * @brief Initialize paths or global resources (optional)
     
    void initialize(const char* data_path, const char* demo_path) {
        printf("[runtime] Initialize called.\n");
        printf("[runtime]   Data Path: %s\n", data_path ? data_path : "NULL");
        printf("[runtime]   Demo Path: %s\n", demo_path ? demo_path : "NULL");
    }

    /**
     * @brief Iterates the registry and starts every plugin.
     * * Since plugins now manage their own threads (via k_thread_create in start()),
     * this function simply calls the factory function for each plugin and returns 
     * immediately. It does not block.
     
    void start_all_plugins() {
        printf("[runtime] Starting all plugins...\n");
        
        PluginRegistry& reg = get_plugin_registry();
        
        // Use range-based for loop (relies on PluginRegistry::begin()/end())
        for (const auto& entry : reg) {
            printf("[runtime] Launching plugin: %s\n", entry.name);
            
            // This calls the factory function (e.g., start_offline_imu),
            // which constructs the plugin and spawns its thread.
            entry.start_fn(pb_);
        }
        
        printf("[runtime] All plugins launched. Threads are running.\n");
    }

    /**
     * @brief Deprecated method kept for main.cpp compatibility.
     * In the threaded model, start_all_plugins() already sets everything running.
     
    void run_all_plugins() {
        // No-op: threads are already running.
    }

    /**
     * @brief Cleanup routine.
     
    void shutdown() {
        printf("[runtime] Shutting down...\n");
        // Logic to signal threads to stop would go here
    }

private:
    phonebook_new& pb_;
};

} // namespace ILLIXR

#endif
*/