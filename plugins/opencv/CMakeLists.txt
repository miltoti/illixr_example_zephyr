cmake_minimum_required(VERSION 3.20.0)

# 1. Setup paths
set(OPENCV_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../third-party/opencv)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(opencv_zephyr)
zephyr_linker_sources(SECTIONS src/custom.ld)

# flags to make it work
set(RV64_FLAGS -march=rv64imafdc_zicsr_zifencei -mabi=lp64d)

add_compile_options(${RV64_FLAGS})
add_link_options(${RV64_FLAGS})
target_compile_options(app PRIVATE ${RV64_FLAGS})
target_link_options(app PRIVATE ${RV64_FLAGS})

# -----------------------------------------------------------------------------
# Cross-compile / Zephyr quirks
# -----------------------------------------------------------------------------
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY CACHE STRING "" FORCE)

if(NOT DEFINED CMAKE_SIZEOF_VOID_P)
  set(CMAKE_SIZEOF_VOID_P 8 CACHE STRING "Pointer size" FORCE) 
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# OpenCV build options (static + minimal)
# -----------------------------------------------------------------------------
set(BUILD_opencv_apps OFF CACHE BOOL "" FORCE)
set(BUILD_opencv_version OFF CACHE BOOL "" FORCE)
set(OPENCV_BUILD_APP_LIST "" CACHE STRING "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_opencv_world OFF CACHE BOOL "" FORCE)

# Make the module list configurable. By default we build a minimal set that is
# appropriate for bare-metal/Zephyr (core,imgproc,imgcodecs). HighGUI requires
# a windowing/GUI backend (GTK/Qt/X11) which is typically not available on
# embedded Zephyr targets. Use -DOPENCV_BUILD_HIGHGUI=ON when building for a
# host/desktop to include the highgui module.
option(OPENCV_BUILD_HIGHGUI "Enable building opencv_highgui module (desktop only)" OFF)
set(OPENCV_DEFAULT_BUILD_LIST "core,imgproc,imgcodecs")
if(OPENCV_BUILD_HIGHGUI)
  set(BUILD_LIST "${OPENCV_DEFAULT_BUILD_LIST},highgui" CACHE STRING "" FORCE)
  set(BUILD_opencv_highgui ON CACHE BOOL "" FORCE)
else()
  set(BUILD_LIST "${OPENCV_DEFAULT_BUILD_LIST}" CACHE STRING "" FORCE)
  set(BUILD_opencv_highgui OFF CACHE BOOL "" FORCE)
endif()
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_PERF_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_DOCS OFF CACHE BOOL "" FORCE)

set(WITH_TIFF OFF CACHE BOOL "" FORCE)
set(BUILD_TIFF OFF CACHE BOOL "" FORCE)
set(WITH_OPENEXR OFF CACHE BOOL "" FORCE)
set(WITH_JASPER OFF CACHE BOOL "" FORCE)
set(WITH_ADE OFF CACHE BOOL "" FORCE)
set(BUILD_ADE OFF CACHE BOOL "" FORCE)

set(OPENCV_FORCE_3RDPARTY_BUILD ON CACHE BOOL "" FORCE)
set(BUILD_ZLIB ON CACHE BOOL "" FORCE)
set(BUILD_PNG ON CACHE BOOL "" FORCE)
set(BUILD_JPEG ON CACHE BOOL "" FORCE)

# Bare-metal RISC-V usually lacks pthread support
set(OPENCV_DISABLE_THREAD_SUPPORT ON CACHE BOOL "" FORCE)
set(WITH_PTHREADS_PF OFF CACHE BOOL "" FORCE)
set(OPENCV_DISABLE_PTHREADS_PF ON CACHE BOOL "" FORCE)

add_compile_definitions(
  M_PI=3.14159265358979323846
  M_E=2.71828182845904523536
  OPENCV_DISABLE_THREAD_SUPPORT
  OPENCV_DISABLE_PTHREADS_PF
)

add_subdirectory(${OPENCV_SRC_DIR} ${CMAKE_CURRENT_BINARY_DIR}/opencv_build)

set_target_properties(zephyr_final PROPERTIES 
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/zephyr"
    OUTPUT_NAME "zephyr"
)

if(TARGET zephyr_prebuilt)
    set_target_properties(zephyr_prebuilt PROPERTIES 
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/zephyr"
    )
endif()

# Ensure the build scripts look for the right filename
set(KERNEL_ELF_NAME zephyr.elf CACHE INTERNAL "" FORCE)

# zephyr
target_sources(app PRIVATE
  src/main.cpp
  src/atomic_shim.c
)

target_link_libraries(app PRIVATE
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
)

# Optional highgui linkage (only when the module is built)
if(OPENCV_BUILD_HIGHGUI)
  target_link_libraries(app PRIVATE opencv_highgui)
endif()

add_dependencies(app opencv_core opencv_imgproc opencv_imgcodecs)
if(OPENCV_BUILD_HIGHGUI)
  add_dependencies(app opencv_highgui)
endif()

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OPENCV_SRC_DIR}/modules/core/include
  ${OPENCV_SRC_DIR}/modules/imgproc/include
  ${OPENCV_SRC_DIR}/modules/imgcodecs/include
  ${CMAKE_CURRENT_BINARY_DIR}/opencv_build
)