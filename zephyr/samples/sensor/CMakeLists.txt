# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

# The library is the module providing mtl::zephyr; point Zephyr at it
# when it is not in the west manifest
set(MTL_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
list(APPEND ZEPHYR_EXTRA_MODULES ${MTL_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mtl_sensor LANGUAGES CXX)

target_sources(app PRIVATE src/main.cpp)

# "west build -t dot" writes the graphs of all three tables into the
# build directory for tools/fsmview
add_custom_target(dot
    COMMAND ${PYTHON_EXECUTABLE} ${MTL_ROOT}/tools/dotgen/dotgen.py
            ${CMAKE_CURRENT_LIST_DIR}/src -o ${CMAKE_BINARY_DIR}
    COMMENT "Writing the state machine graphs into ${CMAKE_BINARY_DIR}"
    VERBATIM
)
