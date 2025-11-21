#ifndef ILLIXR_RUNTIME_HPP
#define ILLIXR_RUNTIME_HPP

#include <string>
#include "phonebook_new.hpp"
#include "node.hpp"

namespace ILLIXR {

class Runtime {
public:
    explicit Runtime(phonebook_new& pb);

    void initialize(const char* data_path, const char* demo_data_path);
    void start_all_plugins();
    void shutdown();

private:
    phonebook_new& pb_;
    std::string data_path_;
    std::string demo_data_;
};

} // namespace ILLIXR

#endif
