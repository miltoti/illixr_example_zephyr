get_filename_component(PLUGIN_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

add_library(${PLUGIN_NAME} OBJECT 
  plugin.cpp
)

target_compile_options(${PLUGIN_NAME} PRIVATE
  "SHELL:-include ${ILLIXR_HELPER_DIR}/opencv_helper.hpp"
  "SHELL:-include ${ILLIXR_HELPER_DIR}/eigen_lib_fix.hpp"
)


target_link_libraries(${PLUGIN_NAME} PRIVATE
  zephyr_interface
  illixr_opencv
)

target_include_directories(${PLUGIN_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    ${ZEPHYR_BASE}/../modules/lib/eigen

)
