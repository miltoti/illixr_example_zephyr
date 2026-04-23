
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdint.h> // For uint64_t
#include "phonebook_new.hpp"
#include "runtime.hpp"

extern "C" {
    extern volatile uint64_t tohost;
    extern volatile uint64_t fromhost;
}

void firesim_exit(int code) {
    // This will now use the same 'tohost' variable that the UART driver uses
    tohost = (uint64_t)((code << 1) | 1);
    while (1); 
}
// --------------------------

uint64_t g_program_start_mtime = 0;

int main() {
    printf("========== ILLIXR Static Runtime (Zephyr) ==========\n");

    ILLIXR::init_phonebook_global();
    auto& pb = ILLIXR::get_phonebook();
    ILLIXR::Runtime runtime(pb);

    const char* data_path      = "../data/V1_02_medium/mav0";
    const char* demo_data_path = "../demo_data";

    runtime.initialize(data_path, demo_data_path);
    runtime.start_all_plugins();
    // INcluding the runtime thread itself timing. Need better way to figure out runtime whole time
    k_sleep(K_SECONDS(500000));

    runtime.shutdown();

    printf("========== Runtime Complete ==========\n");
    
    // Final step: Signal FireSim to terminate
    firesim_exit(0); 

    return 0;
}