/*
 * Transition tracing: fsm::tracing<DERIVED> reports every state change of
 * the machine it is injected into as short type names - the input of a
 * log line. Suitable for target code: the names are compile-time strings
 * in static storage (mtl::short_name_of), so a deferred logger needs no
 * copies. Inject it after the observers whose effects the line should
 * follow.
 *
 * The derived class provides sinks, each optional (a missing sink is
 * simply not called, so a logger can compile them out by log level):
 *   void traceInitial(char const* machine, char const* state);
 *   void traceTransition(char const* machine, char const* from,
 *                        char const* event, char const* to);
 *
 * Line grammar - the contract with tools/fsmview, which ignores anything
 * before "fsm[" (a logger's timestamp and module prefix):
 *   fsm[<machine>] -> <state>                  initial state, on construction
 *   fsm[<machine>] <from> -(<event>)-> <to>    a fired transition
 * <machine> is the short name of the machine's transition table: give the
 * table a name (struct my_table : fsm::transition_table<...> {}) - an
 * alias reads "transition_table". <to> is internal_target when the state
 * handled the event in place, <from> is any_state when a wildcard
 * transition fired through the machine's shared body: tracing declares
 * source_agnostic, so it never forces the per-source expansion. The
 * trace_format constants spell the grammar for std::format and printf;
 * the MTL_FSM_TRACE_*_PRINTF macros are the printf forms as string
 * literals, for loggers that paste the format (Zephyr's LOG_INF).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/StateMachine.hpp>
#include <mtl/TypeName.hpp>
#include <mtl/Typelist.hpp>

#include <string_view>
#include <type_traits>

#define MTL_FSM_TRACE_INITIAL_PRINTF    "fsm[%s] -> %s"
#define MTL_FSM_TRACE_TRANSITION_PRINTF "fsm[%s] %s -(%s)-> %s"

namespace fsm {

namespace trace_format {

inline constexpr std::string_view initial    = "fsm[{}] -> {}";
inline constexpr std::string_view transition = "fsm[{}] {} -({})-> {}";

inline constexpr char const* initial_printf    = MTL_FSM_TRACE_INITIAL_PRINTF;
inline constexpr char const* transition_printf = MTL_FSM_TRACE_TRANSITION_PRINTF;

} // namespace trace_format

template<typename DERIVED>
struct tracing {
    static constexpr bool source_agnostic = true;

    // Construction only: the shared wildcard path enters from any_state,
    // and the unsatisfied constraint keeps this hook out of the
    // shareability probe for real source states
    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
        requires std::is_same_v<OLD_STATE, mtl::nil_type>
    void onEnterState(MACHINE&)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (requires { self.traceInitial(machine<MACHINE>(), name<NEW_STATE>()); }) {
            self.traceInitial(machine<MACHINE>(), name<NEW_STATE>());
        }
    }

    template<typename FROM_STATE, typename EVENT, typename TO_STATE, typename MACHINE>
    void onTransition(MACHINE&)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (requires {
                          self.traceTransition(machine<MACHINE>(), name<FROM_STATE>(),
                                               name<EVENT>(), name<TO_STATE>());
                      }) {
            self.traceTransition(machine<MACHINE>(), name<FROM_STATE>(), name<EVENT>(),
                                 name<TO_STATE>());
        }
    }

private:
    template<typename T>
    static constexpr char const* name()
    {
        return mtl::short_name_of<T>;
    }

    template<typename MACHINE>
    static constexpr char const* machine()
    {
        return name<typename MACHINE::table>();
    }
};

} // namespace fsm
