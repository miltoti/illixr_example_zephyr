#include "phonebook_new.hpp"
#include <new>

namespace ILLIXR {

// Allocate raw storage for the phonebook instance
static uint8_t pb_storage[sizeof(phonebook_new)]
    __attribute__((aligned(alignof(phonebook_new))));

static phonebook_new* pb_ptr = nullptr;

void init_phonebook_global() {
    if (pb_ptr == nullptr) {
        printf("[phonebook] init_phonebook_global(): constructing phonebook at %p\n",
               (void*)pb_storage);

        pb_ptr = new (pb_storage) phonebook_new();

        printf("[phonebook] init_phonebook_global(): pb_ptr = %p\n", (void*)pb_ptr);
    }
}

phonebook_new& get_phonebook() {
    printf("[phonebook] get_phonebook() called\n");
    printf("  pb_ptr = %p\n", (void*)pb_ptr);

    if (pb_ptr == nullptr) {
        printf("[phonebook] ERROR: pb_ptr is NULL! Calling init_phonebook_global() implicitly.\n");
        init_phonebook_global();
    }

    return *pb_ptr;
}

} // namespace ILLIXR
