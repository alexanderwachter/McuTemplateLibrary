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

enum class mode { eco, fast };
struct level {
    int value;
    constexpr bool operator==(level const&) const = default;
};
// not structural (a private member): shown by its type name
class secret {
public:
    constexpr secret() = default;
    constexpr bool operator==(secret const&) const = default;

private:
    int hidden = 0;
};

struct off {
    static constexpr auto annotations = fsm::annotate(mode::eco, level{3}, secret{});
    static constexpr std::string_view dot_note = "Power: Default";
};
struct running {
    static constexpr auto timeout = 50ms;
    static constexpr std::string_view dot_note = "Power: On";
    void handle(go const&) {}
};

struct ready {
    static bool check() { return true; }
};

struct escaped {
    static constexpr std::string_view dot_note = "a <b> & c";
};

// named: the short name is the machine id in the table comment
struct example_table : fsm::transition_table<
    fsm::transition<fsm::from<off>,            fsm::on<go>,           fsm::to<running>,
                    fsm::guard<ready>>,
    fsm::transition<fsm::from<running>,        fsm::on<fsm::timeout>, fsm::to<off>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>,         fsm::to<off>>,
    fsm::internal_transition<fsm::from<running>, fsm::on<go>>,
    fsm::transition<fsm::from<off>, fsm::on<fsm::timeout>, fsm::to<escaped>>> {};

// the label strips namespaces, the type name keeps them (both from
// mtl/TypeName.hpp)
static_assert(fsm::internal::label<fsm::timeout>() == "timeout");
static_assert(mtl::type_name<fsm::timeout>() == "fsm::timeout");

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
    fsm::writeDot<example_table>(out, "example");
    auto const dot = out.str();

    check(dot.starts_with("digraph \"example\" {\n    // table: example_table\n"));
    check(dot.contains("\"off\" -> \"running\" [label=\"go\\n[ready]\" id=\"off__go__running__0\"];"));
    // an HTML-like table: the name bold and larger, a rule between the
    // sections name / timeout / annotation set (type name for the
    // non-structural element) / notes, the details left-aligned
    check(dot.contains("\"off\" [label=<<table border=\"0\" cellborder=\"0\" cellspacing=\"0\">"
                       "<tr><td><b><font point-size=\"16\">off</font></b></td></tr><hr/>"
                       "<tr><td align=\"left\">mode::eco</td></tr>"
                       "<tr><td align=\"left\">level{3}</td></tr>"
                       "<tr><td align=\"left\">secret</td></tr>"
                       "<hr/><tr><td align=\"left\">Power: Default</td></tr></table>>];"));
    check(dot.contains("\"running\" [label=<<table border=\"0\" cellborder=\"0\" cellspacing=\"0\">"
                       "<tr><td><b><font point-size=\"16\">running</font></b></td></tr><hr/>"
                       "<tr><td align=\"left\">timeout 50 ms</td></tr>"
                       "<hr/><tr><td align=\"left\">Power: On</td></tr></table>>];"));
    // markup characters in a note are escaped
    check(dot.contains("<tr><td align=\"left\">a &lt;b&gt; &amp; c</td></tr>"));
    check(dot.contains("\"running\" -> \"off\" [label=\"timeout\" id=\"running__timeout__off__1\"];"));
    check(dot.contains("\"any_state\" [style=dashed];"));
    check(dot.contains("\"any_state\" -> \"off\" [label=\"kill\" id=\"any_state__kill__off__2\"];"));
    check(dot.contains("\"running\" -> \"running\" [label=\"go\\n(internal)\" "
                       "id=\"running__go__internal_target__3\" style=dashed];"));
    check(dot.contains("__initial -> \"off\";"));
    check(dot.ends_with("}\n"));

    return failures;
}
