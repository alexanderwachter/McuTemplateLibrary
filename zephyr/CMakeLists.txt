# Copyright (c) 2026 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_MTL)

zephyr_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# The state machine's debug checks follow Zephyr's assertions (the
# library itself keys them on NDEBUG, which Zephyr never defines)
zephyr_compile_definitions(MTL_FSM_CHECKS=$<IF:$<BOOL:${CONFIG_ASSERT}>,1,0>)

if(CONFIG_MTL_FSM_TRACE)

zephyr_library_named(mtl)

zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/src/TraceLogger.cpp
)

endif()

endif()
