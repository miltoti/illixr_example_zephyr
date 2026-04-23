get_filename_component(PLUGIN_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

add_library(${PLUGIN_NAME} OBJECT plugin.cpp)

# FORCE INJECT the fix header
target_compile_options(${PLUGIN_NAME} PRIVATE 
    -include "${CMAKE_CURRENT_SOURCE_DIR}/../../src/helper/eigen_lib_fix.hpp"
)

target_link_libraries(${PLUGIN_NAME} PRIVATE zephyr_interface)
target_include_directories(${PLUGIN_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    ${ZEPHYR_BASE}/../modules/lib/eigen
)