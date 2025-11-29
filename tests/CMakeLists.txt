#  Copyright (c) 2025 Alexander Wachter
# 
#  SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

project(TemplateMetaProgrammingTests VERSION 0.0.1
        DESCRIPTION "Test for the mtl library"
        LANGUAGES CXX)


#find_package(mtl CONFIG REQUIRED)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/algorithms.cpp
    ${CMAKE_CURRENT_LIST_DIR}/typelist.cpp
    ${CMAKE_CURRENT_LIST_DIR}/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_link_libraries(${PROJECT_NAME}
    mtl
)
