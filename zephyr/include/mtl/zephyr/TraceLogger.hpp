/*
 * Observer logging every transition of the machine it is injected into
 * on the mtl_fsm log module (CONFIG_MTL_FSM_TRACE, info level), in the
 * fsm::tracing line grammar read by tools/fsmview:
 *
 *   mtl::zephyr::TraceLogger trace_logger;
 *   Sink sink{tcpc, vbus, timer, driver, trace_logger};
 *
 * Inject it after the observers whose effects the line should follow.
 * The names are compile-time strings in static storage, so deferred
 * logging needs no duplication. Below info level the sinks are absent:
 * fsm::tracing then calls nothing, and no call site or name string is
 * emitted.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/StateMachineTrace.hpp>

#include <zephyr/logging/log.h>

namespace mtl::zephyr {

// Out of line: the log module lives in TraceLogger.cpp
void logInitialState(char const* machine, char const* state);
void logTransition(char const* machine, char const* from, char const* event, char const* to);

struct TraceLogger : fsm::tracing<TraceLogger> {
#if defined(CONFIG_MTL_FSM_TRACE) && CONFIG_MTL_FSM_LOG_LEVEL >= LOG_LEVEL_INF
    void traceInitial(char const* machine, char const* state)
    {
        logInitialState(machine, state);
    }

    void traceTransition(char const* machine, char const* from, char const* event, char const* to)
    {
        logTransition(machine, from, event, to);
    }
#endif
};

} // namespace mtl::zephyr
