cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(illixr_working)

# ============================================================
# === Global toolchain / cross-compile settings (for OpenCV) ==
# ============================================================

# Make OpenCV's try_compile not attempt to run executables
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY CACHE STRING "" FORCE)

# ============================================================
# === Top-level path anchors ================================
# ============================================================

# Zephyr repo root (…/zephyr_ws/zephyr) is in ZEPHYR_BASE.
# Define a stable workspace top one level above it.
get_filename_component(ZEPHYR_TOP "${ZEPHYR_BASE}" DIRECTORY)

# This Zephyr variable is the root of your application (illixr_working).
# (Zephyr sets it; we fall back safely if needed.)
if(NOT DEFINED APPLICATION_SOURCE_DIR)
  set(APPLICATION_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

message(STATUS "ZEPHYR_BASE            = ${ZEPHYR_BASE}")
message(STATUS "ZEPHYR_TOP             = ${ZEPHYR_TOP}")
message(STATUS "APPLICATION_SOURCE_DIR = ${APPLICATION_SOURCE_DIR}")

# Convenience: absolute path to your helper directory
set(ILLIXR_HELPER_DIR "${APPLICATION_SOURCE_DIR}/src/helper")



# OpenCV's "bitness" detection often fails under Zephyr cross builds
if(NOT DEFINED CMAKE_SIZEOF_VOID_P)
  set(CMAKE_SIZEOF_VOID_P 8 CACHE STRING "Pointer size" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# === RISC-V flags (apply to EVERYTHING: app + plugins + OpenCV)
# ============================================================

set(RV64_FLAGS -march=rv64imafdc_zicsr_zifencei -mabi=lp64d)

# Apply globally so subprojects (OpenCV) inherit them
add_compile_options(${RV64_FLAGS})
add_link_options(${RV64_FLAGS})

# ============================================================
# === OpenCV: build from source once at top-level ============
# ============================================================

# Path to OpenCV source
set(OPENCV_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../third-party/opencv)

# If your OpenCV needs linker script fragments, apply at top-level (final link)
zephyr_linker_sources(SECTIONS
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/opencv/src/custom.ld
)

# OpenCV build options (static + minimal; matches your working cmake)
set(BUILD_opencv_apps OFF CACHE BOOL "" FORCE)
set(BUILD_opencv_version OFF CACHE BOOL "" FORCE)
set(OPENCV_BUILD_APP_LIST "" CACHE STRING "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_opencv_world OFF CACHE BOOL "" FORCE)

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
set(WITH_OPENJPEG OFF CACHE BOOL "" FORCE)
set(BUILD_OPENJPEG OFF CACHE BOOL "" FORCE)

# Bare-metal: no pthread support
set(OPENCV_DISABLE_THREAD_SUPPORT ON CACHE BOOL "" FORCE)
set(WITH_PTHREADS_PF OFF CACHE BOOL "" FORCE)
set(OPENCV_DISABLE_PTHREADS_PF ON CACHE BOOL "" FORCE)

# IMPORTANT:
# Keep these as GLOBAL compile definitions so OpenCV subproject sees them
add_compile_definitions(
  OPENCV_DISABLE_THREAD_SUPPORT
  OPENCV_DISABLE_PTHREADS_PF
  M_PI=3.14159265358979323846
  M_E=2.71828182845904523536
)

# Build OpenCV in the main build tree
add_subdirectory(${OPENCV_SRC_DIR} ${CMAKE_CURRENT_BINARY_DIR}/opencv_build)

# ============================================================
# === OpenCV support shim (compile ONCE) =====================
# ============================================================

# atomic_shim.c provides missing atomics / builtins for OpenCV under Zephyr.
# Compile it once to avoid multiple-definition problems.
add_library(illixr_opencv_support STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/opencv/src/atomic_shim.c
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/opencv/src/libatomic_exchange_1_shim.c
)

target_link_libraries(illixr_opencv_support PRIVATE zephyr_interface)

# ============================================================
# === OpenCV bundle target used by app + plugins =============
# ============================================================

add_library(illixr_opencv INTERFACE)

# Anything linking illixr_opencv gets these include dirs and libs
set(OPENCV_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv_build)

target_include_directories(illixr_opencv INTERFACE
  ${OPENCV_BIN_DIR}/include          # <--- THIS is the big missing one
  ${CMAKE_CURRENT_BINARY_DIR}        # <--- covers build/opencv2 (seen in your tree)
  ${OPENCV_SRC_DIR}/modules/core/include
  ${OPENCV_SRC_DIR}/modules/imgproc/include
  ${OPENCV_SRC_DIR}/modules/imgcodecs/include
)

target_link_libraries(illixr_opencv INTERFACE
  -Wl,--start-group
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  illixr_opencv_support
  -Wl,--end-group
)

if(OPENCV_BUILD_HIGHGUI)
  target_link_libraries(illixr_opencv INTERFACE opencv_highgui)
endif()

# Make OpenCV available to non-plugin app code too
target_link_libraries(app PRIVATE illixr_opencv)

# ============================================================
# === YAML CONFIG PARSING ====================================
# ============================================================

if (NOT DEFINED YAML_FILE)
  message(FATAL_ERROR "You must pass: -DYAML_FILE=<path_to_yaml>")
endif()

get_filename_component(YAML_FILE "${YAML_FILE}" ABSOLUTE)
message(STATUS "Using YAML_FILE: ${YAML_FILE}")

set(READ_YAML_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/read_yaml.py")
set(GENERATED_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/generated_config.hpp")

add_custom_command(
  OUTPUT ${GENERATED_CONFIG}
  COMMAND ${Python3_EXECUTABLE} ${READ_YAML_SCRIPT} ${YAML_FILE} ${GENERATED_CONFIG}
  DEPENDS ${READ_YAML_SCRIPT} ${YAML_FILE}
  COMMENT "Parsing YAML and generating ${GENERATED_CONFIG}"
)

add_custom_target(generate_yaml ALL DEPENDS ${GENERATED_CONFIG})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# ============================================================
# === CORE SOURCE FILES ======================================
# ============================================================

target_sources(app PRIVATE
  src/main.cpp
  src/node.cpp
  src/relative_clock.cpp
  src/phonebook_new.cpp
  src/stoplight.cpp
  data/V1_02_medium/mav0/imu0/data.csv
)

# ============================================================
# === Restore Zephyr output paths (OpenCV can mess with them) =
# ============================================================

set_target_properties(zephyr_final PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/zephyr"
  OUTPUT_NAME "zephyr"
)

if(TARGET zephyr_prebuilt)
  set_target_properties(zephyr_prebuilt PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/zephyr"
  )
endif()

# Ensure Zephyr post-build scripts look for zephyr/zephyr.elf
set(KERNEL_ELF_NAME zephyr.elf CACHE INTERNAL "" FORCE)


# ============================================================
# === READ YAML AND LOAD PLUGIN NAMES ========================
# ============================================================

file(READ "${YAML_FILE}" YAML_CONTENTS)
string(REGEX MATCH "plugins:[ \t]*([^\n\r]+)" PLUGIN_LINE "${YAML_CONTENTS}")

if(PLUGIN_LINE)
  string(REGEX REPLACE "plugins:[ \t]*" "" PLUGIN_LIST_RAW "${PLUGIN_LINE}")
  string(REPLACE "," ";" PLUGIN_NAMES "${PLUGIN_LIST_RAW}")

  set(PLUGIN_NAMES_CLEAN "")
  foreach(P IN LISTS PLUGIN_NAMES)
    string(STRIP "${P}" P_STRIPPED)
    if(P_STRIPPED)
      list(APPEND PLUGIN_NAMES_CLEAN "${P_STRIPPED}")
    endif()
  endforeach()

  message(STATUS "Resolved plugins: ${PLUGIN_NAMES_CLEAN}")

  foreach(P IN LISTS PLUGIN_NAMES_CLEAN)
    set(PLUGIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/plugins/${P}")
    message(STATUS "Adding plugin '${P}' from ${PLUGIN_DIR}")

    add_subdirectory("${PLUGIN_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/plugins/${P}")

    if(TARGET ${P})
      target_sources(app PRIVATE $<TARGET_OBJECTS:${P}>)
      message(STATUS "Added plugin '${P}' object files into app")
    else()
      message(WARNING "Plugin '${P}' did not define target '${P}'")
    endif()
  endforeach()
endif()
