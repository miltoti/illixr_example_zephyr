add_library(p3 STATIC plugin.cpp)

# Let plugin1 use Zephyr functions like printk
target_link_libraries(p3 PRIVATE zephyr_interface)

# Include ILLIXR src headers
target_include_directories(p3 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
