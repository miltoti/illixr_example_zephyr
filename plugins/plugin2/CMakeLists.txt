add_library(plugin2 STATIC plugin.cpp)

target_link_libraries(plugin2 PRIVATE zephyr_interface)

# Include ILLIXR src headers
target_include_directories(plugin2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
