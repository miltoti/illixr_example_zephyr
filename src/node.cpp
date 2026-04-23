// node.cpp

#include "node.hpp"

namespace ILLIXR {

// ---------------------------
// Explicit initialization
// ---------------------------

void Node::initialize(phonebook_new& pb, const char* name) {
    pb_ = &pb;

    // Save name locally
    strncpy(name_, name, MAX_PLUGIN_NAME_LEN - 1);
    name_[MAX_PLUGIN_NAME_LEN - 1] = '\0';

    // NOTE: registration with phonebook is done elsewhere (Plugin wrapper).
}

// ---------------------------
// Periodic service
// ---------------------------

void Node::service_periodic() {
    if (!pb_) { return; }

    uint64_t now_ms = k_uptime_get();
    for (auto& job : periodic_jobs_) {
        job->tick(now_ms);
    }
}

// ---------------------------
// start() is virtual and has
// no base implementation.
// ---------------------------

} // namespace ILLIXR
