/*
 * Graphviz DOT rendering of a transition table, for visualizing and
 * documenting state machines. Development tooling for hosted builds -
 * not meant for target code.
 *
 * States become nodes whose label is sectioned by rules: the name, the
 * timeout, the elements of the state's annotation set (fsm::annotate) as
 * the compiler spells their values (color::red, lamp{true}; a
 * non-structural element by its type name), and a state's optional
 * static dot_note and dot_action strings. Every transition is one
 * labeled edge (guards in brackets), the initial state gets an entry
 * marker, a final state a double border, and an any_state wildcard
 * source is shown as a dashed node. A state emitting an event to the
 * machine above names it in its label. A
 * composite state names its submachine's table in its label; the
 * sub-table is a graph of its own (write it separately - tools/dotgen
 * finds every named table).
 *
 * For tools/fsmview the graph carries a "// table: <short name>" comment
 * naming the table (the machine id of fsm::tracing lines), one
 * "// submachine: <state> <table>" comment per composite state (the
 * child graph a parent's exit deactivates), and every edge an id
 * "<from>__<event>__<to>__<index>" that Graphviz passes into its SVG
 * output; the index in the table keeps guarded alternatives of one
 * (state, event) pair distinct, <to> is internal_target for internal
 * transitions - the same names the trace lines use.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <mtl/StateMachine.hpp>
#include <mtl/TypeName.hpp>
#include <mtl/Typelist.hpp>

#include <chrono>
#include <cstddef>
#include <ostream>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace fsm {

namespace internal {

// Namespace qualifiers and template arguments dropped for readable labels
template<typename T>
constexpr std::string_view label()
{
    return mtl::short_name<T>();
}

// Text inside an HTML-like label: the markup characters escaped
inline void writeHtmlText(std::ostream& out, std::string_view text)
{
    for (char const c : text) {
        switch (c) {
        case '<': out << "&lt;"; break;
        case '>': out << "&gt;"; break;
        case '&': out << "&amp;"; break;
        default: out << c;
        }
    }
}

// One row of the node's label table: the detail rows left-aligned,
// the name row centered, bold and larger
inline void writeDotRow(std::ostream& out, std::string_view text)
{
    out << "<tr><td align=\"left\">";
    writeHtmlText(out, text);
    out << "</td></tr>";
}

inline void writeDotNameRow(std::ostream& out, std::string_view name)
{
    out << "<tr><td><b><font point-size=\"16\">";
    writeHtmlText(out, name);
    out << "</font></b></td></tr>";
}

// One annotation-set element: its value when the compiler can spell it
// (a structural type usable as a template argument), else its type
template<typename STATE, std::size_t INDEX>
void writeDotAnnotation(std::ostream& out)
{
    if constexpr (requires { mtl::value_name<std::get<INDEX>(STATE::annotations.values)>(); }) {
        writeDotRow(out, mtl::short_value_name<std::get<INDEX>(STATE::annotations.values)>());
    } else {
        writeDotRow(out, label<mtl::at_t<INDEX, annotation_types_t<STATE>>>());
    }
}

template<typename STATE>
void writeDotAnnotations(std::ostream& out)
{
    [&out]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
        (writeDotAnnotation<STATE, INDEXs>(out), ...);
    }(std::make_index_sequence<mtl::count_v<annotation_types_t<STATE>>>{});
}

// The label is an HTML-like table so that a rule separates the
// sections: the name, the timeout, the submachine, the event the state
// emits, the annotation set, the notes. A final state is drawn with a
// double border
template<typename STATE>
void writeDotNode(std::ostream& out, bool final_state)
{
    constexpr bool timed     = has_timeout_v<STATE>;
    constexpr bool nesting   = internal::composite<STATE>;
    constexpr bool emitting  = internal::emitting<STATE>;
    constexpr bool annotated = internal::annotated<STATE>;
    constexpr bool noted     = requires { std::string_view{STATE::dot_note}; };
    constexpr bool acting    = requires { std::string_view{STATE::dot_action}; };
    constexpr bool labelled  = timed || nesting || emitting || annotated || noted || acting;

    out << "    \"" << label<STATE>() << '"';
    if (labelled || final_state) {
        out << " [";
    }
    if constexpr (labelled) {
        out << "label=<<table border=\"0\" cellborder=\"0\" cellspacing=\"0\">";
        writeDotNameRow(out, label<STATE>());
        if constexpr (timed) {
            auto const ms = std::chrono::ceil<std::chrono::milliseconds>(STATE::timeout).count();
            out << "<hr/><tr><td align=\"left\">timeout " << ms << " ms</td></tr>";
        }
        if constexpr (nesting) {
            out << "<hr/>";
            writeDotRow(out, "submachine " + std::string{label<submachine_t<STATE>>()});
        }
        if constexpr (emitting) {
            out << "<hr/>";
            writeDotRow(out, "emits " + std::string{label<emitted_t<STATE>>()});
        }
        if constexpr (annotated) {
            out << "<hr/>";
            writeDotAnnotations<STATE>(out);
        }
        if constexpr (noted || acting) {
            out << "<hr/>";
        }
        if constexpr (noted) {
            writeDotRow(out, std::string_view{STATE::dot_note});
        }
        if constexpr (acting) {
            writeDotRow(out, std::string_view{STATE::dot_action});
        }
        out << "</table>>";
        if (final_state) {
            out << ' ';
        }
    }
    if (final_state) {
        out << "peripheries=2";
    }
    if (labelled || final_state) {
        out << ']';
    }
    out << ";\n";
}

// The row's condition: its parts joined by &&, an inverted part with !
template<typename PART>
void writeDotGuard(std::ostream& out, bool first)
{
    if (!first) {
        out << " && ";
    }
    if constexpr (internal::is_negated_v<PART>) {
        out << '!';
    }
    out << label<internal::guard_of_t<PART>>();
}

template<typename... PARTs>
void writeDotGuards(std::ostream& out, mtl::typelist<PARTs...>)
{
    bool first = true;
    ((writeDotGuard<PARTs>(out, first), first = false), ...);
}

template<std::size_t INDEX, typename TRANSITION>
void writeDotEdge(std::ostream& out)
{
    // an internal transition renders as a dashed self-edge
    using to = std::conditional_t<is_internal_v<TRANSITION>, typename TRANSITION::from,
                                  typename TRANSITION::to>;
    out << "    \"" << label<typename TRANSITION::from>() << "\" -> \"" << label<to>()
        << "\" [label=\"" << label<typename TRANSITION::event>();
    if constexpr (has_guard_v<TRANSITION>) {
        out << "\\n[";
        writeDotGuards(out, typename TRANSITION::guards{});
        out << ']';
    }
    if constexpr (is_internal_v<TRANSITION>) {
        out << "\\n(internal)";
    }
    out << "\" id=\"" << label<typename TRANSITION::from>() << "__"
        << label<typename TRANSITION::event>() << "__" << label<typename TRANSITION::to>()
        << "__" << INDEX << '"';
    if constexpr (is_internal_v<TRANSITION>) {
        out << " style=dashed";
    }
    out << "];\n";
}

template<typename STATES, typename TRANSITIONS, typename FINAL_STATES>
struct dot_writer;

// One comment per composite state, naming the graph its child lives
// in: how tools/fsmview knows which graph a parent's exit deactivates
template<typename STATE>
void writeDotSubmachine(std::ostream& out)
{
    if constexpr (composite<STATE>) {
        out << "    // submachine: " << label<STATE>() << ' ' << label<submachine_t<STATE>>()
            << '\n';
    }
}

template<typename... STATEs, typename... TRANSITIONs, typename FINAL_STATES>
struct dot_writer<mtl::typelist<STATEs...>, mtl::typelist<TRANSITIONs...>, FINAL_STATES> {
    static void writeSubmachines(std::ostream& out) { (writeDotSubmachine<STATEs>(out), ...); }

    static void write(std::ostream& out)
    {
        (writeDotNode<STATEs>(out, mtl::has_a_v<FINAL_STATES, STATEs>), ...);
        [&out]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
            (writeDotEdge<INDEXs, TRANSITIONs>(out), ...);
        }(std::index_sequence_for<TRANSITIONs...>{});
    }

    static constexpr bool uses_wildcard =
        (std::is_same_v<typename TRANSITIONs::from, any_state> || ...);
};

} // namespace internal

template<concepts::transition_table TABLE>
void writeDot(std::ostream& out, std::string_view name = "fsm")
{
    using writer  = internal::dot_writer<typename TABLE::states, typename TABLE::transitions,
                                         typename TABLE::final_states>;
    using initial = mtl::front_t<typename TABLE::states>;

    out << "digraph \"" << name << "\" {\n"
        << "    // table: " << internal::label<TABLE>() << '\n';
    writer::writeSubmachines(out);
    out << "    rankdir=LR;\n"
        << "    label=\"" << name << "\";\n"
        << "    labelloc=t;\n"
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
