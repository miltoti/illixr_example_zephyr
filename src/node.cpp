#include "node.hpp"

namespace ILLIXR {

// ---------------------------
// Node Constructor Behavior
// ---------------------------
// - Node() and Node(name) DO NOT register with phonebook.
// - Only initialize() registers the plugin.
// - This avoids global static constructor usage before Zephyr boot.

// Default constructor already covered in header.
// Name-only constructor already covered in header.

// ---------------------------
// Explicit initialization
// ---------------------------

void Node::initialize(phonebook_new& pb, const char* name) {
    pb_ = &pb;

    // Save name locally
    strncpy(name_, name, MAX_PLUGIN_NAME_LEN - 1);
    name_[MAX_PLUGIN_NAME_LEN - 1] = '\0';

    // Register with phonebook now that RTOS is running
    pb_->register_plugin(name_, this);
}


void Node::shutdown() {
    running_ = false;

    // optionally wake threads, drain queues, etc
}

// ---------------------------
// Virtual start()
// ---------------------------
// No implementation needed; plugins override start().

} // namespace ILLIXR
