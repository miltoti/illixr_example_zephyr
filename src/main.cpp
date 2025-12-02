#include <zephyr/kernel.h>
#include <stdio.h>

#include "phonebook_new.hpp"
#include "runtime.hpp"

int main() {
    printf("========== ILLIXR Static Runtime (Zephyr) ==========\n");

    // 1. Construct the global phonebook (safe AFTER kernel boot)
    //*
    ILLIXR::init_phonebook_global();
    auto& pb = ILLIXR::get_phonebook();

    // 2. Construct runtime using the phonebook
    ILLIXR::Runtime runtime(pb);

    // 3. Paths (const char*, not std::string)
    const char* data_path      = "../data/V1_02_medium/mav0";
    const char* demo_data_path = "../demo_data";

    // 4. Initialize runtime
    runtime.initialize(data_path, demo_data_path);

    // 5. Start all plugins
    runtime.start_all_plugins();

    runtime.run_all_plugins();

    // 6. Sleep for run duration
    k_sleep(K_SECONDS(50));

    // 7. Shutdown
    runtime.shutdown();

    printf("========== Runtime Complete ==========\n");
    return 0;
}
