/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/StateMachineTrace.hpp>

#include <format>
#include <print>
#include <source_location>
#include <string>
#include <vector>

namespace {

struct go {};
struct tick {};
struct kill {
    int code;
};

struct idle {
    int ticks = 0;
    void handle(tick const&) { ++ticks; }
};
struct busy {};
struct dead {
    dead() = default;
    explicit dead(kill const& event) : code(event.code) {}
    int code = 0;
};

// named: the short name is the machine id of every line
struct trace_table : fsm::transition_table<
    fsm::transition<fsm::from<idle>, fsm::on<go>, fsm::to<busy>>,
    fsm::internal_transition<fsm::from<idle>, fsm::on<tick>>,
    fsm::transition<fsm::from<busy>, fsm::on<go>, fsm::to<idle>>,
    fsm::transition<fsm::from<fsm::any_state>, fsm::on<kill>, fsm::to<dead>>> {};

// formats the grammar lines exactly as a logger would
struct line_tracer : fsm::tracing<line_tracer> {
    void traceInitial(char const* machine, char const* state)
    {
        lines.push_back(std::format(fsm::trace_format::initial, machine, state));
    }
    void traceTransition(char const* machine, char const* from, char const* event, char const* to)
    {
        lines.push_back(std::format(fsm::trace_format::transition, machine, from, event, to));
    }
    std::vector<std::string> lines;
};

// the initial sink is optional
struct transition_tracer : fsm::tracing<transition_tracer> {
    void traceTransition(char const*, char const*, char const*, char const*) { ++transitions; }
    int transitions = 0;
};

static_assert(fsm::tracing<line_tracer>::source_agnostic);

int failures = 0;

void check(bool condition, std::source_location location = std::source_location::current())
{
    if (!condition) {
        ++failures;
        std::print("FAILED: {}:{}\n", location.file_name(), location.line());
    }
}

void tracerFormatsEveryKindOfChange()
{
    line_tracer tracer;
    fsm::state_machine<trace_table, line_tracer> sm{tracer};

    check(tracer.lines == std::vector<std::string>{"fsm[trace_table] -> idle"});

    check(sm.process(tick{}));
    check(tracer.lines.back() == "fsm[trace_table] idle -(tick)-> internal_target");
    check(sm.process(go{}));
    check(tracer.lines.back() == "fsm[trace_table] idle -(go)-> busy");

    // the wildcard fires through the shared body (the tracer is
    // source-agnostic): one line, from any_state, no bogus initial line
    check(sm.process(kill{7}));
    check(tracer.lines.size() == 4);
    check(tracer.lines.back() == "fsm[trace_table] any_state -(kill)-> dead");
    check(sm.getIf<dead>()->code == 7);
}

void tracerWithoutInitialSink()
{
    transition_tracer tracer;
    fsm::state_machine<trace_table, transition_tracer> sm{tracer};

    check(tracer.transitions == 0);
    check(sm.process(go{}));
    check(tracer.transitions == 1);
}

} // namespace

int traceTests()
{
    tracerFormatsEveryKindOfChange();
    tracerWithoutInitialSink();
    return failures;
}
