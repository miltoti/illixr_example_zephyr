// plugins/opencv/src/libatomic_exchange_1_shim.c
#include <stdint.h>

// GCC/libatomic ABI for 1-byte exchange.
// Signature used by libstdc++ when std::atomic<uint8_t/bool>::exchange()
// falls back to libatomic.
//
// ptr: address of the 1-byte atomic object
// val: points to 1-byte new value
// old: points to 1-byte where old value is written
// model: memory order (we ignore and use seq-cst for safety)
void __atomic_exchange_1(volatile void* ptr, void* val, void* old, int model) {
    (void)model;

    volatile uint8_t* p8 = (volatile uint8_t*)ptr;
    uint8_t newv = *(uint8_t*)val;

    // We implement byte exchange by doing an atomic CAS loop on the containing 32-bit word.
    uintptr_t addr = (uintptr_t)p8;
    uintptr_t word_addr = addr & ~(uintptr_t)0x3;
    unsigned byte_off = (unsigned)(addr & 0x3);

    volatile uint32_t* p32 = (volatile uint32_t*)word_addr;

    uint32_t shift = (uint32_t)(byte_off * 8u);
    uint32_t mask  = (uint32_t)(0xFFu << shift);

    while (1) {
        // Load containing word
        uint32_t expected = __atomic_load_n(p32, __ATOMIC_SEQ_CST);

        // Extract old byte
        uint8_t oldv = (uint8_t)((expected & mask) >> shift);

        // Form desired word with replaced byte
        uint32_t desired = (expected & ~mask) | ((uint32_t)newv << shift);

        // Try CAS
        if (__atomic_compare_exchange_n(p32, &expected, desired,
                                       0 /*weak*/,
                                       __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {
            *(uint8_t*)old = oldv;
            return;
        }
        // else: expected was updated by the intrinsic, loop again
    }
}
