/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <print>

int statemachineTests();
int dotTests();

int main(int argc, const char* argv[]) {
    int const failures = statemachineTests() + dotTests();
    if (failures != 0) {
        std::print("{} check(s) FAILED\n", failures);
        return 1;
    }
    std::print("all checks passed\n");
    return 0;
}
