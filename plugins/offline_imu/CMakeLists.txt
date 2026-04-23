get_filename_component(PLUGIN_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

# Create a library target with that name
add_library(${PLUGIN_NAME} OBJECT plugin.cpp)

# FORCE INJECT the fix header for all files in this target
target_compile_options(${PLUGIN_NAME} PRIVATE 
    -include "${CMAKE_CURRENT_SOURCE_DIR}/../../src/helper/eigen_lib_fix.hpp"
)

# Let this plugin use Zephyr functions like printk
target_link_libraries(${PLUGIN_NAME} PRIVATE zephyr_interface)

# Include ILLIXR src headers
target_include_directories(${PLUGIN_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    ${ZEPHYR_BASE}/../modules/lib/eigen
)