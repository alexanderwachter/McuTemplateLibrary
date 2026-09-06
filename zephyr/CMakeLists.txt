# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_MTL)

zephyr_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_LIST_DIR}/include
)

if(CONFIG_MTL_FSM_TRACE)

zephyr_library_named(mtl)

zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/src/TraceLogger.cpp
)

endif()

endif()
