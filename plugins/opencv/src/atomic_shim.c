// src/atomic_shim.c
#include <stdint.h>

#ifndef __riscv_atomic
// You really want the 'A' extension; your -march has 'a' already.
// This guard is just to make the failure obvious if it ever changes.
#warning "RISC-V atomics (A extension) not detected"
#endif

#include <stdint.h>

/* * GCC expects __atomic_exchange_1 to return the old value directly.
 * The signature mismatch in your log was:
 * Expected: unsigned char(volatile void *, unsigned char, int)
 */
unsigned char __atomic_exchange_1(volatile void *mem, unsigned char val, int model) {
    (void)model; // Memory model ignored for this simple shim

    volatile uint8_t *p8 = (volatile uint8_t *)mem;
    
    // Calculate word alignment for LR.W / SC.W
    uintptr_t addr = (uintptr_t)p8;
    uintptr_t word_addr = addr & ~(uintptr_t)0x3;
    unsigned shift = (unsigned)((addr & 0x3) * 8);

    volatile uint32_t *p32 = (volatile uint32_t *)word_addr;

    uint32_t loaded, desired;
    uint32_t tmp;

    do {
        // Load the 32-bit word containing our byte
        __asm__ volatile (
            "lr.w %0, (%1)\n"
            : "=r"(loaded)
            : "r"(p32)
            : "memory"
        );

        // Mask out the old byte and insert the new one
        uint32_t mask = 0xFFu << shift;
        desired = (loaded & ~mask) | ((uint32_t)val << shift);

        // Attempt to store the modified word
        __asm__ volatile (
            "sc.w %0, %2, (%1)\n"
            : "=r"(tmp)
            : "r"(p32), "r"(desired)
            : "memory"
        );
    } while (tmp != 0); // Retry if the reservation was broken

    // Extract and return the old byte value
    return (unsigned char)((loaded >> shift) & 0xFFu);
}