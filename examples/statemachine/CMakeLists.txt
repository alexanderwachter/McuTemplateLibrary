#  Copyright (c) 2026 Alexander Wachter
#
#  SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

project(StateMachineExample VERSION 0.0.1
        DESCRIPTION "Traffic light example for the fsm state machine"
        LANGUAGES CXX)

add_executable(TrafficLightExample
    ${CMAKE_CURRENT_LIST_DIR}/traffic_light.cpp
)

target_compile_features(TrafficLightExample PRIVATE cxx_std_23)
target_link_libraries(TrafficLightExample
    mtl
)
