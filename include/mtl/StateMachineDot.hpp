/*
 * Graphviz DOT rendering of a transition table, for visualizing and
 * documenting state machines. Development tooling for hosted builds -
 * not meant for target code.
 *
 * States become nodes (timeouts annotated, and a state's optional
 * static dot_note and dot_action strings appended to its label), every
 * transition one labeled edge (guards in brackets), the initial state
 * gets an entry marker, and an any_state wildcard source is shown as a
 * dashed node.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/StateMachine.hpp>
#include <mtl/Typelist.hpp>

#include <chrono>
#include <ostream>
#include <string_view>
#include <type_traits>

namespace fsm {

namespace internal {

template<typename T>
constexpr std::string_view type_name()
{
#if defined(__GNUC__) || defined(__clang__)
    std::string_view const function = __PRETTY_FUNCTION__;
    auto const start = function.find("T = ") + 4;
    return function.substr(start, function.find_first_of("];", start) - start);
#else
    return "unknown";
#endif
}

// Namespace qualifiers dropped for readable labels
template<typename T>
constexpr std::string_view label()
{
    auto const name  = type_name<T>();
    auto const colon = name.rfind("::");
    return colon == std::string_view::npos ? name : name.substr(colon + 2);
}

template<typename STATE>
void writeDotNode(std::ostream& out)
{
    constexpr bool timed  = has_timeout_v<STATE>;
    constexpr bool noted  = requires { std::string_view{STATE::dot_note}; };
    constexpr bool acting = requires { std::string_view{STATE::dot_action}; };

    out << "    \"" << label<STATE>() << '"';
    if constexpr (timed || noted || acting) {
        out << " [label=\"" << label<STATE>();
        if constexpr (timed) {
            auto const ms = std::chrono::ceil<std::chrono::milliseconds>(STATE::timeout).count();
            out << "\\ntimeout " << ms << " ms";
        }
        if constexpr (noted) {
            out << "\\n" << std::string_view{STATE::dot_note};
        }
        if constexpr (acting) {
            out << "\\n" << std::string_view{STATE::dot_action};
        }
        out << "\"]";
    }
    out << ";\n";
}

template<typename TRANSITION>
void writeDotEdge(std::ostream& out)
{
    // an internal transition renders as a dashed self-edge
    using to = std::conditional_t<is_internal_v<TRANSITION>, typename TRANSITION::from,
                                  typename TRANSITION::to>;
    out << "    \"" << label<typename TRANSITION::from>() << "\" -> \"" << label<to>()
        << "\" [label=\"" << label<typename TRANSITION::event>();
    if constexpr (has_guard_v<TRANSITION>) {
        out << "\\n[" << label<typename TRANSITION::guard>() << ']';
    }
    if constexpr (is_internal_v<TRANSITION>) {
        out << "\\n(internal)\" style=dashed];\n";
    } else {
        out << "\"];\n";
    }
}

template<typename STATES, typename TRANSITIONS>
struct dot_writer;

template<typename... STATEs, typename... TRANSITIONs>
struct dot_writer<mtl::typelist<STATEs...>, mtl::typelist<TRANSITIONs...>> {
    static void write(std::ostream& out)
    {
        (writeDotNode<STATEs>(out), ...);
        (writeDotEdge<TRANSITIONs>(out), ...);
    }

    static constexpr bool uses_wildcard =
        (std::is_same_v<typename TRANSITIONs::from, any_state> || ...);
};

} // namespace internal

template<concepts::transition_table TABLE>
void writeDot(std::ostream& out, std::string_view name = "fsm")
{
    using writer  = internal::dot_writer<typename TABLE::states, typename TABLE::transitions>;
    using initial = mtl::front_t<typename TABLE::states>;

    out << "digraph \"" << name << "\" {\n"
        << "    rankdir=LR;\n"
        << "    node [shape=box, style=rounded];\n"
        << "    __initial [shape=point];\n";
    if constexpr (writer::uses_wildcard) {
        out << "    \"" << internal::label<any_state>() << "\" [style=dashed];\n";
    }
    writer::write(out);
    out << "    __initial -> \"" << internal::label<initial>() << "\";\n"
        << "}\n";
}

} // namespace fsm
