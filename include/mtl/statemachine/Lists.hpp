/*
 * fsm: the lists an author writes - each named for what it lists, its
 * elements declared by their concept
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Feature.hpp>
#include <mtl/statemachine/Observing.hpp>
#include <mtl/statemachine/Timeout.hpp>
#include <mtl/statemachine/Transition.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

namespace fsm {

// The context types a state declares for itself (contexts) or a
// composite state for its submachine (parent_contexts)
template<concepts::context... CONTEXTs>
using contexts = mtl::typelist<CONTEXTs...>;

// The annotation types an observer declares it observes
template<concepts::annotation... ANNOTATIONs>
using annotations = mtl::typelist<ANNOTATIONs...>;

// The events a state owes a transition for
template<concepts::event... EVENTs>
using events = mtl::typelist<EVENTs...>;

// The observers of a machine, for the traits asking what they enable
// together
template<concepts::observer... OBSERVERs>
using observers = mtl::typelist<OBSERVERs...>;

// A timer-range map. Maps compose like the tables they describe
template<concepts::timed_by_or_timer_ranges... TIMED_BY_OR_TIMER_RANGEs>
using timer_ranges = mtl::linearize_t<mtl::typelist<TIMED_BY_OR_TIMER_RANGEs...>>;

} // namespace fsm
