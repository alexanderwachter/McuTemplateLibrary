/*
 * fsm: the contexts of a machine - its own instances, the ones it
 * inherits from its parent machine, and what its states are
 * constructed with
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Table.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

namespace fsm::internal {

// Every context type the states of TABLE declare, deduplicated, in
// order of first appearance
template<concepts::transition_table TABLE>
using table_contexts_t =
    mtl::unique_t<mtl::linearize_t<mtl::transform_t<typename TABLE::states, contexts_of>>>;

// A composite state names the contexts of its own machine that its
// submachine inherits:
//   using parent_contexts = fsm::contexts<line_status>;
// A sub-state declaring an inherited type binds to the parent
// machine's instance instead of a fresh one - the same lifetime as the
// parent's; every other context type of the sub-table is the child's
// own, fresh on each entry of the composite. An inherited type the
// submachine does not use itself may be inherited further down through
// a composite of the sub-table
template<typename STATE>
concept declares_parent_contexts =
    concepts::state<STATE> && requires { typename STATE::parent_contexts; } &&
    mtl::concepts::typelist<typename STATE::parent_contexts>;

template<concepts::state STATE>
struct parent_contexts_of : std::type_identity<mtl::typelist<>> {};

template<declares_parent_contexts STATE>
struct parent_contexts_of<STATE> : std::type_identity<typename STATE::parent_contexts> {};

template<concepts::state STATE>
using parent_contexts_t = typename parent_contexts_of<STATE>::type;

// The checks on a parent_contexts declaration, each a trait so a
// failing one can be asked per state: only a composite has a child to
// inherit; the child inherits what the machine holds (CONTEXTS: the
// machine's own and inherited contexts); and some state of the
// submachine declares every inherited type - one nobody uses is a dead
// declaration
template<concepts::state STATE>
struct parent_contexts_on_composite
    : std::bool_constant<!declares_parent_contexts<STATE> || composite<STATE>> {};

template<mtl::concepts::typelist CONTEXTS>
struct parent_contexts_held_in {
    template<concepts::state STATE>
    struct pred : std::bool_constant<mtl::all_of_v<parent_contexts_t<STATE>,
                                                   member_of<CONTEXTS>::template pred>> {};
};

// The contexts declared by the states of a composite's submachine, at
// any level: what its child can usefully inherit
template<concepts::state STATE>
struct submachine_contexts : std::type_identity<mtl::typelist<>> {};

template<composite STATE>
struct submachine_contexts<STATE>
    : std::type_identity<mtl::unique_t<mtl::linearize_t<
          mtl::transform_t<all_states_t<submachine_t<STATE>>, contexts_of>>>> {};

template<concepts::state STATE>
struct parent_contexts_declared_in_submachine
    : std::bool_constant<
          mtl::all_of_v<parent_contexts_t<STATE>,
                        member_of<typename submachine_contexts<STATE>::type>::template pred>> {};

// The contexts a machine inherits from its parent: none for a root
template<concepts::transition_table TABLE>
struct inherited_contexts : std::type_identity<mtl::typelist<>> {};

template<concepts::transition_table TABLE, std::size_t DEPTH, mtl::concepts::typelist INHERITED>
struct inherited_contexts<nested<TABLE, DEPTH, INHERITED>> : std::type_identity<INHERITED> {};

template<concepts::transition_table TABLE>
using inherited_contexts_t = typename inherited_contexts<TABLE>::type;

// What an element of a machine's context storage is aligned to: an
// inherited context is held as a reference, a pointer in storage
template<typename ELEMENT>
inline constexpr std::size_t stored_alignment_v = alignof(ELEMENT);

template<typename T>
inline constexpr std::size_t stored_alignment_v<T&> = alignof(T*);

template<typename LHS, typename RHS>
struct stored_no_less_aligned
    : std::bool_constant<(stored_alignment_v<LHS> >= stored_alignment_v<RHS>)> {};

// The contexts of one machine. DECLARED_CONTEXTS: what the states of
// its table declare; INHERITED_CONTEXTS: what its parent machine
// hands down - held as references to the parent's instances, every
// other declared type as an instance of the machine's own
template<mtl::concepts::typelist DECLARED_CONTEXTS, mtl::concepts::typelist INHERITED_CONTEXTS>
class MachineContexts {
public:
    using inherited_contexts = INHERITED_CONTEXTS;
    using own_contexts =
        mtl::remove_if_t<DECLARED_CONTEXTS, member_of<inherited_contexts>::template pred>;
    using context_types = mtl::concat_t<own_contexts, inherited_contexts>;

    // What the child machine of COMPOSITE inherits
    template<composite COMPOSITE>
    using inherited_by = parent_contexts_t<COMPOSITE>;

    // A root's
    MachineContexts() = default;

    // A child machine's, inheriting from its parent machine's
    template<mtl::concepts::typelist PARENT_DECLARED, mtl::concepts::typelist PARENT_INHERITED>
    explicit MachineContexts(MachineContexts<PARENT_DECLARED, PARENT_INHERITED>& parent)
        : MachineContexts(parent, std::type_identity<context_tuple>{})
    {
    }

    // A copy would share the own instances' values, not inherit
    MachineContexts(MachineContexts const&)            = delete;
    MachineContexts& operator=(MachineContexts const&) = delete;

    // --- what the table must satisfy ----------------------------------------

    static constexpr bool own_contexts_default_constructible =
        mtl::all_of_v<own_contexts, std::is_default_constructible>;

    // In their declared order, without any other argument
    template<concepts::transition_table TABLE>
    static constexpr bool every_state_constructible_from_its_contexts =
        mtl::all_of_v<typename TABLE::states, context_constructible>;

    template<concepts::transition_table TABLE>
    static constexpr bool parent_contexts_only_on_composites =
        mtl::all_of_v<typename TABLE::states, parent_contexts_on_composite>;

    // Own or inherited in turn
    template<concepts::transition_table TABLE>
    static constexpr bool holds_parent_contexts_of_every_composite =
        mtl::all_of_v<mtl::filter_t<typename TABLE::states, is_composite>,
                      parent_contexts_held_in<context_types>::template pred>;

    // By a state of the submachine or of the submachines inside it
    template<concepts::transition_table TABLE>
    static constexpr bool every_submachine_declares_its_parent_contexts =
        mtl::all_of_v<mtl::filter_t<typename TABLE::states, is_composite>,
                      parent_contexts_declared_in_submachine>;

    // --- the instances ------------------------------------------------------

    template<concepts::context T>
    [[nodiscard]] T& context()
    {
        return contextIn<T>(contexts_);
    }

    template<concepts::context T>
    [[nodiscard]] T const& context() const
    {
        return contextIn<T>(contexts_);
    }

    // The contexts STATE declares, as its constructor takes them
    template<concepts::state STATE>
    auto contextsOf()
    {
        return [this]<typename... CONTEXTs>(mtl::typelist<CONTEXTs...>) {
            return std::tie(contextIn<CONTEXTs>(contexts_)...);
        }(contexts_of_t<STATE>{});
    }

    // Arguments constructing STATE in place inside a variant: the
    // in_place tag and its contexts. The tuple round-trip through
    // make_from_tuple is free: its prvalue is elided into the variant
    // (measured GCC 15 -Os: direct stores, no tuple, no move)
    template<concepts::state STATE>
    auto initialArgumentsOf()
    {
        return [this]<typename... CONTEXTs>(mtl::typelist<CONTEXTs...>) {
            return std::forward_as_tuple(std::in_place_type<STATE>,
                                         contextIn<CONTEXTs>(contexts_)...);
        }(contexts_of_t<STATE>{});
    }

private:
    using inherited_references =
        mtl::transform_t<inherited_contexts, std::add_lvalue_reference>;
    // The most aligned first: no padding between them
    using stored_contexts =
        mtl::sort_t<mtl::concat_t<own_contexts, inherited_references>, stored_no_less_aligned>;
    using context_tuple = mtl::rebind_t<stored_contexts, std::tuple>;

    template<typename PARENT, typename... ELEMENTs>
    MachineContexts(PARENT& parent, std::type_identity<std::tuple<ELEMENTs...>>)
        : contexts_(element<ELEMENTs>(parent)...)
    {
    }

    // A fresh own instance, or the reference to the parent's
    template<typename ELEMENT, typename PARENT>
    static ELEMENT element(PARENT& parent)
    {
        if constexpr (std::is_reference_v<ELEMENT>) {
            return parent.template context<std::remove_reference_t<ELEMENT>>();
        } else {
            return ELEMENT{};
        }
    }

    // An own instance held by value, or the parent machine's behind
    // the inherited reference
    template<typename T, typename CONTEXT_TUPLE>
    static constexpr auto& contextIn(CONTEXT_TUPLE& contexts)
    {
        if constexpr (mtl::has_a_v<inherited_contexts, T>) {
            return std::get<T&>(contexts);
        } else {
            return std::get<T>(contexts);
        }
    }

    context_tuple contexts_{};
};

// The contexts of the machine built from TABLE (a child machine's
// comes wrapped in internal::nested) and running ENABLED_TABLE
template<concepts::transition_table TABLE, concepts::transition_table ENABLED_TABLE>
using machine_contexts_t =
    MachineContexts<table_contexts_t<ENABLED_TABLE>, inherited_contexts_t<TABLE>>;

} // namespace fsm::internal
