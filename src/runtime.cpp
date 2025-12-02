#include "runtime.hpp"
#include "plugin_registry.hpp"
#include "phonebook_new.hpp"
#include <zephyr/kernel.h>
#include <stdio.h>
#include "relative_clock.hpp"

namespace ILLIXR {

// Thread settings for per-plugin threads
constexpr size_t PLUGIN_STACK_SIZE = 4096;
constexpr int    PLUGIN_PRIORITY   = 5;

K_THREAD_STACK_ARRAY_DEFINE(plugin_stacks, MAX_PLUGINS, PLUGIN_STACK_SIZE);
static struct k_thread plugin_threads[MAX_PLUGINS];

// Thread entry: each plugin runs its Node::start() here
static void plugin_thread_entry(void* p1, void*, void*) {
    Node* node = static_cast<Node*>(p1);
    if (!node) {
        printf("[runtime] plugin_thread_entry: NULL node\n");
        return;
    }

    printf("[runtime] Plugin '%s' starting in its own thread...\n", node->name());
    node->start();
    printf("[runtime] Plugin '%s' exited.\n", node->name());
}

Runtime::Runtime(phonebook_new& pb)
    : pb_{pb}
{
}

void Runtime::initialize(const char* data_path, const char* demo_data_path) {
    if (data_path) {
        data_path_ = data_path;
    }
    if (demo_data_path) {
        demo_data_ = demo_data_path;
    }
    printf("[runtime] Initialized with data='%s', demo_data='%s'\n",
           data_path_.c_str(), demo_data_.c_str());
}

void Runtime::start_all_plugins() {
    printf("[runtime] Starting all plugins...\n");

    // 1. Ask the plugin registry to construct all plugins.
    //
    // Each registered plugin_start_fn_t will:
    //   static PluginType instance(pb);
    //   instance.start();   // start() will set up periodic publishers, etc.
    //
    // During construction, each plugin should call
    //   initialize(pb, "plugin_name");
    // which registers it into pb_.
    for (auto& entry : get_plugin_registry()) {
        entry.start_fn(pb_);
    }

    // 2. Now phonebook_new should contain all Node instances.
    //    Spawn a Zephyr thread per plugin.
    size_t idx = 0;
    printf("test\n");
    printf("%d\n", pb_.size());
    for (auto it = pb_.begin(); it != pb_.end(); ++it) {
        if (idx >= MAX_PLUGINS) {
            printf("[runtime] WARNING: MAX_PLUGINS reached, skipping extras\n");
            break;
        }

        Node* node = it->instance;
        if (!node) {
            continue;
        }

        printf("[runtime] Launching plugin '%s' on thread %zu\n",
               it->name, idx);

        k_thread_create(&plugin_threads[idx],
                        plugin_stacks[idx],
                        K_THREAD_STACK_SIZEOF(plugin_stacks[idx]),
                        plugin_thread_entry,
                        node, nullptr, nullptr,
                        PLUGIN_PRIORITY, 0, K_NO_WAIT);
        ++idx;
    }

    printf("[runtime] All plugins launched.\n");

    auto& rc = get_global_relative_clock();
    rc.start();
    rc.print();

    printf("[runtime] Global RelativeClock started\n");
}

void Runtime::run_all_plugins() {
    printf("[runtime] run_all_plugins() called\n");
    // In this static runtime, all plugins are already started in their own threads.
    // This function can be used for additional runtime management if needed.
}

void Runtime::shutdown() {
    printf("[runtime] Shutting down...\n");

    size_t n = pb_.size();
    for (size_t i = 0; i < n && i < MAX_PLUGINS; i++) {
        k_thread_join(&plugin_threads[i], K_FOREVER);
    }

    printf("[runtime] All plugin threads stopped.\n");
}

} // namespace ILLIXR
