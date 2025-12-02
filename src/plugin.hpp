// src/plugin.hpp
#pragma once

#include "node.hpp"
#include "phonebook_new.hpp"

namespace ILLIXR {

/**
 * Minimal Plugin base:
 *  - Contains a Node.
 *  - Constructor calls node_.initialize(pb, name).
 *  - Does NOT own threads. Runtime still runs Node::start() in its own threads.
 *
 * This is just to avoid repeating the "Node node_; node_.initialize(...)" pattern.
 */
class Plugin {
public:
    Plugin(phonebook_new& pb, const char* name)
        : node_{} {
        node_.initialize(pb, name);  // register with phonebook_new
    }

    virtual ~Plugin() = default;

    // Access to the underlying Node for pub/sub.
    Node& node()       { return node_; }
    const Node& node() const { return node_; }

private:
    Node node_;
};

} // namespace ILLIXR
