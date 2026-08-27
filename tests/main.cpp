/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <print>

int statemachine_tests();

int main(int argc, const char* argv[]) {
    int const failures = statemachine_tests();
    if (failures != 0) {
        std::print("{} check(s) FAILED\n", failures);
        return 1;
    }
    std::print("all checks passed\n");
    return 0;
}
