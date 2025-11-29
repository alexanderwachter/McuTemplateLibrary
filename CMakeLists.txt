
# Copyright (c) 2025 Alexander Wachter
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)

project("McuTemplateLibrary" VERSION 0.0.1
        DESCRIPTION "A header-only C++ library for type-save hardware access "
        LANGUAGES CXX
        HOMEPAGE_URL "https://github.com/alexanderwachter/McuTemplateLibrary")

set(TARGET_NAME mtl)

add_library(${TARGET_NAME} INTERFACE)
target_compile_features(${TARGET_NAME} INTERFACE cxx_std_20)

target_include_directories(${TARGET_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

option(ENABLE_TESTING "Enable the tests" ${PROJECT_IS_TOP_LEVEL})
if(ENABLE_TESTING)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tests)
endif()
