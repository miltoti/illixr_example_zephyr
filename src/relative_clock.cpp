#include "relative_clock.hpp"

#include <cstdio>  // std::printf

namespace ILLIXR {

RelativeClock& get_global_relative_clock() {
    static RelativeClock clock;
    return clock;
}

void RelativeClock::print() const {
    auto ns = now_ns();
    std::printf("relative_clock now: %lld ns since start\n",
                static_cast<long long>(ns));
}

} // namespace ILLIXR
