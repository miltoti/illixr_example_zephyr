# Get the folder name of this plugin, e.g. "p3" or "offline_imu"
get_filename_component(PLUGIN_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

# Create a library target with that name
add_library(${PLUGIN_NAME} OBJECT plugin.cpp)

# Let this plugin use Zephyr functions like printk
target_link_libraries(${PLUGIN_NAME} PRIVATE zephyr_interface)

# Include ILLIXR src headers
target_include_directories(${PLUGIN_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    ${ZEPHYR_BASE}/../modules/lib/eigen

)
