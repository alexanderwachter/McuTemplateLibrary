/*
 * fsm: value observation - annotation sets and the fsm::observing base
 * with compile-time change suppression
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <concepts>
#include <tuple>
#include <type_traits>
#include <utility>

namespace fsm {

namespace internal {

template<typename OBSERVER, typename STATE>
inline constexpr bool observes_v = requires { OBSERVER::template annotation<STATE>(); };

// Whether STATE's annotation differs from OTHER's: an annotation OTHER
// lacks, or of another type, always counts as a change; an unobserved
// STATE never notifies
template<typename OBSERVER, typename STATE, typename OTHER>
struct annotation_changes : std::bool_constant<observes_v<OBSERVER, STATE>> {};

template<typename OBSERVER, typename STATE, typename OTHER>
    requires observes_v<OBSERVER, STATE> && observes_v<OBSERVER, OTHER> &&
             std::same_as<decltype(OBSERVER::template annotation<STATE>()),
                          decltype(OBSERVER::template annotation<OTHER>())>
struct annotation_changes<OBSERVER, STATE, OTHER>
    : std::bool_constant<OBSERVER::template annotation<STATE>() !=
                         OBSERVER::template annotation<OTHER>()> {};

template<typename OBSERVER, typename STATE, typename OTHER>
inline constexpr bool annotation_changes_v = annotation_changes<OBSERVER, STATE, OTHER>::value;

} // namespace internal

// Several annotations on one state, keyed by their types:
//
//   struct green {
//       static constexpr auto annotations = fsm::annotate(led_pattern::on, power::high);
//   };
//
// An fsm::observing observer then needs no observe_static(): its
// notifyEntry(led_pattern) / notifyExit(power) overloads pick the
// elements they consume, each with its own change suppression. The type
// is the key, so the types of one set are distinct and meaningful (a
// strong type per annotation, not int)
template<typename... Ts>
struct annotation_set {
    static_assert(std::is_same_v<mtl::unique_t<mtl::typelist<Ts...>>, mtl::typelist<Ts...>>,
                  "fsm::annotate: the annotation types of one state must be distinct");

    using types = mtl::typelist<Ts...>;

    template<typename T>
    static constexpr bool has = (std::is_same_v<T, Ts> || ...);

    template<typename T>
        requires has<T>
    constexpr T const& get() const
    {
        return std::get<T>(values);
    }

    std::tuple<Ts...> values;
};

template<typename... Ts>
constexpr annotation_set<std::remove_cvref_t<Ts>...> annotate(Ts&&... values)
{
    return {{std::forward<Ts>(values)...}};
}

namespace internal {

template<typename T>
struct is_annotation_set : std::false_type {};

template<typename... Ts>
struct is_annotation_set<annotation_set<Ts...>> : std::true_type {};

template<typename STATE>
concept annotated = requires {
    STATE::annotations;
    requires is_annotation_set<std::remove_cvref_t<decltype(STATE::annotations)>>::value;
};

// The element types of a state's set, none for a state without one
template<typename STATE>
struct annotation_types : std::type_identity<mtl::typelist<>> {};

template<annotated STATE>
struct annotation_types<STATE>
    : std::type_identity<typename std::remove_cvref_t<decltype(STATE::annotations)>::types> {};

template<typename STATE>
using annotation_types_t = typename annotation_types<STATE>::type;

// Lazy: the set is only named for annotated states (a plain && would
// substitute both operands)
template<typename STATE, typename T>
struct has_annotation : std::false_type {};

template<annotated STATE, typename T>
struct has_annotation<STATE, T>
    : std::bool_constant<std::remove_cvref_t<decltype(STATE::annotations)>::template has<T>> {};

template<typename STATE, typename T>
inline constexpr bool has_annotation_v = has_annotation<STATE, T>::value;

// Whether STATE's T element differs from OTHER's: an element OTHER lacks
// always counts as a change, a state without the element never notifies
template<typename T, typename STATE, typename OTHER>
struct set_annotation_changes : std::bool_constant<has_annotation_v<STATE, T>> {};

template<typename T, typename STATE, typename OTHER>
    requires has_annotation_v<STATE, T> && has_annotation_v<OTHER, T>
struct set_annotation_changes<T, STATE, OTHER>
    : std::bool_constant<STATE::annotations.template get<T>() !=
                         OTHER::annotations.template get<T>()> {};

template<typename T, typename STATE, typename OTHER>
inline constexpr bool set_annotation_changes_v = set_annotation_changes<T, STATE, OTHER>::value;

// Whether STATE's T element reaches one of OBSERVER's hooks
template<typename OBSERVER, typename STATE>
struct element_notified {
    template<typename T>
    struct pred : std::bool_constant<requires(OBSERVER observer) {
                      observer.notifyEntry(STATE::annotations.template get<T>());
                  } || requires(OBSERVER observer) {
                      observer.notifyExit(STATE::annotations.template get<T>());
                  }> {};
};

template<typename OBSERVER, typename STATE>
inline constexpr bool set_notified_v =
    mtl::any_of_v<annotation_types_t<STATE>, element_notified<OBSERVER, STATE>::template pred>;

} // namespace internal

// Value observer base: the derived class names the watched member once and
// provides notifyEntry(value) (new state's value) and/or notifyExit(value)
// (old state's value, old state still alive), each optional:
//
//   struct lamp_driver : fsm::observing<lamp_driver> {
//       template<typename STATE>
//       static constexpr auto observe_static() -> decltype(STATE::lamps)
//       {
//           return STATE::lamps;
//       }
//       void notifyEntry(lamps_t const& lamps);
//   };
//
// The trailing return type makes states without the member drop out via
// SFINAE. The change check runs at compile time: edges between equal
// values emit no code.
//
// observe_static() is for static constexpr members: the value is read at
// type level - states need not be constructible, and naming a non-static
// member is a compile error rather than a silently wrong probe value.
// Equal-value edges are elided at compile time. For a non-static member
// (e.g. an event payload delivered into the state) provide
// observe_nonstatic() instead: it reads the current state instance, and
// the notify hooks run on every edge into/out of an observing state -
// per-instance values cannot be change-suppressed:
//
//   struct phy_driver : fsm::observing<phy_driver> {
//       static constexpr auto observe_nonstatic(auto const& state)
//           -> decltype((state.tx_message))
//       {
//           return state.tx_message;
//       }
//       void notifyEntry(message_t const& message); // hand to hardware
//   };
//
// An observer may declare both; on the same edge the static hook is
// guaranteed to run before the nonstatic one. This ordering is part of
// the contract: a static annotation can prepare (e.g. reset) what the
// nonstatic observation then consumes.
//
// A state's annotation set (fsm::annotate above) is observed without
// any observe declaration: every element type with a notifyEntry /
// notifyExit overload is delivered, change-suppressed per element, in
// the set's order, between the static and the nonstatic hook.
//
// Both hook forms: where the edge is known (every exact edge, and the
// exit side of a wildcard) the edge form suppresses a value that does
// not change between the two states, at compile time; the entry side
// of a wildcard has no edge, so the machine takes the one-state form,
// which notifies every value of the state entered - the one place a
// value observer sees a re-notification
template<typename DERIVED>
struct observing {

    template<typename STATE>
    static constexpr auto annotation()
        requires requires { DERIVED::template observe_static<STATE>(); }
    {
        return DERIVED::template observe_static<STATE>();
    }

    // The edge form. The static path is per edge (the change check needs
    // both states); the nonstatic path delegates to one body per observed
    // state
    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onExitFrom(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (internal::annotation_changes_v<DERIVED, OLD_STATE, NEW_STATE>) {
            if constexpr (requires { self.notifyExit(DERIVED::template annotation<OLD_STATE>()); }) {
                self.notifyExit(DERIVED::template annotation<OLD_STATE>());
            }
        }
        this->template setExit<OLD_STATE, NEW_STATE>(internal::annotation_types_t<OLD_STATE>{});
        this->template nonstaticExit<OLD_STATE>(machine);
    }

    template<typename OLD_STATE, typename NEW_STATE, typename MACHINE>
    void onEnterFrom(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (internal::annotation_changes_v<DERIVED, NEW_STATE, OLD_STATE>) {
            if constexpr (requires { self.notifyEntry(DERIVED::template annotation<NEW_STATE>()); }) {
                self.notifyEntry(DERIVED::template annotation<NEW_STATE>());
            }
        }
        this->template setEnter<OLD_STATE, NEW_STATE>(internal::annotation_types_t<NEW_STATE>{});
        this->template nonstaticEnter<NEW_STATE>(machine);
    }

    // The one-state form: no other state to compare against, which is the
    // edge form against mtl::nil_type - every value counts as a change
    template<typename STATE, typename MACHINE>
    void onExit(MACHINE& machine)
    {
        this->template onExitFrom<STATE, mtl::nil_type>(machine);
    }

    template<typename STATE, typename MACHINE>
    void onEnter(MACHINE& machine)
    {
        this->template onEnterFrom<mtl::nil_type, STATE>(machine);
    }

private:
    // The set elements, each on its own change check
    template<typename OLD_STATE, typename NEW_STATE, typename... Ts>
    void setExit(mtl::typelist<Ts...>)
    {
        auto& self = static_cast<DERIVED&>(*this);
        ([&] {
            if constexpr (internal::set_annotation_changes_v<Ts, OLD_STATE, NEW_STATE>) {
                if constexpr (requires { self.notifyExit(OLD_STATE::annotations.template get<Ts>()); }) {
                    self.notifyExit(OLD_STATE::annotations.template get<Ts>());
                }
            }
        }(), ...);
    }

    template<typename OLD_STATE, typename NEW_STATE, typename... Ts>
    void setEnter(mtl::typelist<Ts...>)
    {
        auto& self = static_cast<DERIVED&>(*this);
        ([&] {
            if constexpr (internal::set_annotation_changes_v<Ts, NEW_STATE, OLD_STATE>) {
                if constexpr (requires { self.notifyEntry(NEW_STATE::annotations.template get<Ts>()); }) {
                    self.notifyEntry(NEW_STATE::annotations.template get<Ts>());
                }
            }
        }(), ...);
    }

    template<typename STATE, typename MACHINE>
    void nonstaticExit(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (requires {
                          self.notifyExit(
                              DERIVED::observe_nonstatic(*machine.template getIf<STATE>()));
                      }) {
            // The machine is in STATE here, so getIf() cannot fail. The check
            // is what stops GCC reporting a potential null dereference inside
            // <variant> once this inlines at -Os.
            if (auto* const state = machine.template getIf<STATE>(); state != nullptr) {
                self.notifyExit(DERIVED::observe_nonstatic(*state));
            }
        }
    }

    template<typename STATE, typename MACHINE>
    void nonstaticEnter(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (requires {
                          self.notifyEntry(
                              DERIVED::observe_nonstatic(*machine.template getIf<STATE>()));
                      }) {
            // The machine is in STATE here, so getIf() cannot fail. The check
            // is what stops GCC reporting a potential null dereference inside
            // <variant> once this inlines at -Os.
            if (auto* const state = machine.template getIf<STATE>(); state != nullptr) {
                self.notifyEntry(DERIVED::observe_nonstatic(*state));
            }
        }
    }
};

} // namespace fsm
