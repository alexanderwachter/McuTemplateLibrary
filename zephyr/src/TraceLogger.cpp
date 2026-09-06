/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/zephyr/TraceLogger.hpp>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mtl_fsm, CONFIG_MTL_FSM_LOG_LEVEL);

namespace mtl::zephyr {

// The log macros paste the format string: it has to be a literal
void logInitialState(char const* machine, char const* state)
{
    LOG_INF(MTL_FSM_TRACE_INITIAL_PRINTF, machine, state);
}

void logTransition(char const* machine, char const* from, char const* event, char const* to)
{
    LOG_INF(MTL_FSM_TRACE_TRANSITION_PRINTF, machine, from, event, to);
}

} // namespace mtl::zephyr
