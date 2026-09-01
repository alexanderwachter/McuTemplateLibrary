/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachineDot.hpp>

#include <chrono>
#include <print>
#include <source_location>
#include <sstream>
#include <string>
#include <string_view>

using namespace std::chrono_literals;

namespace {

struct go {};
struct kill {};

struct off {
    static constexpr std::string_view dot_note = "Power: Default";
};
struct running {
    static constexpr auto timeout = 50ms;
    static constexpr std::string_view dot_note = "Power: On";
};

struct ready {
    static bool check() { return true; }
};

using table = fsm::transition_table<
    fsm::transition<fsm::from<off>,            fsm::on<go>,           fsm::to<running>,
                    fsm::guard<ready>>,
    fsm::transition<fsm::from<running>,        fsm::on<fsm::timeout>, fsm::to<off>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>,         fsm::to<off>>>;

// the label strips namespaces, the type name keeps them
static_assert(fsm::internal::label<fsm::timeout>() == "timeout");
static_assert(fsm::internal::type_name<fsm::timeout>() == "fsm::timeout");

int failures = 0;

void check(bool condition, std::source_location location = std::source_location::current())
{
    if (!condition) {
        ++failures;
        std::print("FAILED: {}:{}\n", location.file_name(), location.line());
    }
}

} // namespace

int dotTests()
{
    std::ostringstream out;
    fsm::writeDot<table>(out, "example");
    auto const dot = out.str();

    check(dot.starts_with("digraph \"example\" {"));
    check(dot.contains("\"off\" -> \"running\" [label=\"go\\n[ready]\"];"));
    check(dot.contains("\"off\" [label=\"off\\nPower: Default\"];"));
    check(dot.contains("\"running\" [label=\"running\\ntimeout 50 ms\\nPower: On\"];"));
    check(dot.contains("\"running\" -> \"off\" [label=\"timeout\"];"));
    check(dot.contains("\"any_state\" [style=dashed];"));
    check(dot.contains("\"any_state\" -> \"off\" [label=\"kill\"];"));
    check(dot.contains("__initial -> \"off\";"));
    check(dot.ends_with("}\n"));

    return failures;
}
