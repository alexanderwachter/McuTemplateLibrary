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

# "west build -t dot" writes the graphs into the build directory for
# tools/fsmview: led_table from the crawl, and the sensor table as the
# variant this build's configuration selects (the table is a template
# keyed by the observers, so it is named explicitly)
if(CONFIG_SAMPLE_CALIBRATION)
    set(SENSOR_TABLE "sensor::sensor_table<sensor::every_feature>")
else()
    set(SENSOR_TABLE "sensor::sensor_table<>")
endif()
add_custom_target(dot
    COMMAND ${PYTHON_EXECUTABLE} ${MTL_ROOT}/tools/dotgen/dotgen.py
            ${CMAKE_CURRENT_LIST_DIR}/src/led.hpp ${CMAKE_CURRENT_LIST_DIR}/src/sensor.hpp
            --table "${SENSOR_TABLE}=sensor_table" -o ${CMAKE_BINARY_DIR}
    COMMENT "Writing the state machine graphs into ${CMAKE_BINARY_DIR}"
    VERBATIM
)
