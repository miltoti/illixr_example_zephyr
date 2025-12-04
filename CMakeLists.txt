# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(illixr_working)

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
    #src/runtime.cpp
    src/relative_clock.cpp
    src/phonebook_new.cpp
)

# ============================================================
# === READ YAML AND LOAD PLUGIN NAMES ========================
# ============================================================

file(READ "${YAML_FILE}" YAML_CONTENTS)

# Extract line: plugins: plugin1,plugin2,...
string(REGEX MATCH "plugins:[ \t]*([^\n\r]+)" PLUGIN_LINE "${YAML_CONTENTS}")

if(PLUGIN_LINE)
    # Remove "plugins:" prefix
    string(REGEX REPLACE "plugins:[ \t]*" "" PLUGIN_LIST_RAW "${PLUGIN_LINE}")

    # Convert to CMake list
    string(REPLACE "," ";" PLUGIN_NAMES "${PLUGIN_LIST_RAW}")

    # Strip whitespace from names
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
            # P is an OBJECT library: pull its .o files directly into app
            target_sources(app PRIVATE $<TARGET_OBJECTS:${P}>)
            message(STATUS "Added plugin '${P}' object files into app")
        else()
            message(WARNING "Plugin '${P}' did not define target '${P}'")
        endif()
    endforeach()
endif()
