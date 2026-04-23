// eigen_lib_fix.hpp
#ifndef EIGEN_LIB_FIX_HPP
#define EIGEN_LIB_FIX_HPP

#include <cmath>

// Fix for Eigen math functions on RISC-V embedded targets
#ifdef __riscv
    // fabsf64 was not found in your toolchain scope
    // Standard fabs handles double and is widely supported in Newlib
    #ifndef fabsl
        #define fabsl(x) fabs((double)(x))
    #endif
#endif

// Ensure M_PI is defined for plugins like the One Euro Filter
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#endif