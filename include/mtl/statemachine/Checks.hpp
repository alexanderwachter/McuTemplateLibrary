/*
 * fsm: the debug checks on a machine's use
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cassert>

// Debug checks on the machine's use: process() re-entered from a hook
// (the contract forbids it: on the wildcard path the exit hooks have
// run but the state has not changed yet), or a state constructor that
// threw and left the machine without a state. On without NDEBUG;
// define MTL_FSM_CHECKS to 0 or 1 to decide explicitly - the same
// value in every translation unit, the flag is a member. The failing
// check goes through MTL_FSM_ASSERT(condition, "message"), assert() by
// default
#ifndef MTL_FSM_CHECKS
#  ifdef NDEBUG
#    define MTL_FSM_CHECKS 0
#  else
#    define MTL_FSM_CHECKS 1
#  endif
#endif

#ifndef MTL_FSM_ASSERT
#  define MTL_FSM_ASSERT(condition, message) assert((condition) && message)
#endif
