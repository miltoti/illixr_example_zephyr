#ifndef ILLIXR_RUNTIME_HPP
#define ILLIXR_RUNTIME_HPP

#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdint.h>
#include "phonebook_new.hpp"
#include "plugin_registry.hpp"
#include "stoplight.hpp"   // extern declarations only — definitions are in stoplight.cpp

// Defined in main.cpp; recorded here at the moment data flow begins.
extern uint64_t g_program_start_mtime;

// CLINT mtime: global real-time counter shared across all harts.
// Address 0x200bff8 from the spike DTS. Safe to compare across CPUs.
static inline uint64_t read_mtime_runtime() {
    volatile uint64_t* mtime = reinterpret_cast<volatile uint64_t*>(0x200bff8UL);
    return *mtime;
}

namespace ILLIXR {

class Runtime {
public:
    explicit Runtime(phonebook_new& pb) : pb_(pb) {}

    void initialize(const char* data_path, const char* demo_path) {
        printf("[runtime] Initialize called.\n");
        printf("[runtime]   Data Path: %s\n", data_path ? data_path : "NULL");
        printf("[runtime]   Demo Path: %s\n", demo_path ? demo_path : "NULL");
    }

    void start_all_plugins() {
        printf("[runtime] Starting all plugins...\n");
        printf("[runtime] Thread: %p\n", k_current_get());

        PluginRegistry& reg = get_plugin_registry();

        // ── Step 1: spawn all plugin threads ─────────────────────────────
        // Order doesn't matter — we wait for all of them below before
        // any data starts flowing.
        for (const auto& entry : reg) {
            printf("[runtime] Launching plugin: %s\n", entry.name);
            entry.start_fn(pb_);
        }

        // ── Step 2: wait for every thread to finish _p_thread_setup() ────
        // Each plugin's threadloop::run() gives stoplight_ready once after
        // _p_thread_setup() returns — meaning all subscriptions are registered.
        // We take reg.size() times so we know every plugin is fully ready
        // before any data starts flowing.
        printf("[runtime] Waiting for %zu plugins to finish setup...\n",
               reg.size());
        for (size_t i = 0; i < reg.size(); i++) {
            k_sem_take(&stoplight_ready, K_FOREVER);
            printf("[runtime] %zu/%zu plugins ready\n", i + 1, reg.size());
        }

        g_program_start_mtime = read_mtime_runtime();
        printf("[runtime] All %zu plugins ready — data flow begins.\n",
               reg.size());
        printf("[runtime] Timing start: mtime=%llu ticks\n",
               (unsigned long long)g_program_start_mtime);
    }

    void shutdown() {
        printf("[runtime] Shutting down...\n");
    }

private:
    phonebook_new& pb_;
};

} // namespace ILLIXR

#endif // ILLIXR_RUNTIME_HPP