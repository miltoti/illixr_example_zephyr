add_library(plugin1 STATIC plugin.cpp)

# Let plugin1 use Zephyr functions like printk
target_link_libraries(plugin1 PRIVATE zephyr_interface)

# Include ILLIXR src headers
target_include_directories(plugin1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
