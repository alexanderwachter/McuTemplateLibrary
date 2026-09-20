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
// strong type per annotation, not int).
//
// The values of an instance - an event payload delivered into the
// state, a computed report - form a set the same way, from a const
// member function named values():
//
//   struct sending {
//       auto values() const { return fsm::annotate_ref(message, orientation()); }
//   };
//
// Delivered on every edge into or out of the state (per-instance values
// cannot be change-suppressed), after the static set. annotate_ref
// keeps lvalues by reference, so a message is not copied into the set;
// a values() returning one value instead of a set counts as a
// one-element set. Elements may be stored by reference: the key is
// always the plain type
template<typename... Ts>
struct annotation_set {
    using types = mtl::typelist<std::remove_cvref_t<Ts>...>;
    static_assert(std::is_same_v<mtl::unique_t<types>, types>,
                  "fsm::annotate: the annotation types of one state must be distinct");

    template<typename T>
    static constexpr bool has = mtl::has_a_v<types, std::remove_cvref_t<T>>;

    template<typename T>
        requires has<T>
    constexpr std::remove_cvref_t<T> const& get() const
    {
        return std::get<mtl::index_of_v<std::remove_cvref_t<T>, types>>(values);
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
using ref_or_value_t = std::conditional_t<std::is_lvalue_reference_v<T>,
                                          std::remove_reference_t<T> const&,
                                          std::remove_cvref_t<T>>;

} // namespace internal

template<typename... Ts>
constexpr annotation_set<internal::ref_or_value_t<Ts>...> annotate_ref(Ts&&... values)
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

// A state with per-instance values; values() may return a set or one
// value, which instanceValues() wraps into a set (by reference when
// values() returns a reference)
template<typename STATE>
concept valued = requires(STATE const& state) { state.values(); };

template<valued STATE>
constexpr auto instanceValues(STATE const& state)
{
    if constexpr (is_annotation_set<std::remove_cvref_t<decltype(state.values())>>::value) {
        return state.values();
    } else {
        return annotate_ref(state.values());
    }
}

template<typename STATE>
struct instance_types : std::type_identity<mtl::typelist<>> {};

template<valued STATE>
struct instance_types<STATE>
    : std::type_identity<typename decltype(instanceValues(std::declval<STATE const&>()))::types> {};

template<typename STATE>
using instance_types_t = typename instance_types<STATE>::type;

template<typename STATE, typename T>
inline constexpr bool has_value_v = mtl::has_a_v<instance_types_t<STATE>, std::remove_cvref_t<T>>;

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

// Whether STATE's T value reaches one of OBSERVER's hooks
template<typename OBSERVER, typename STATE>
struct value_notified {
    template<typename T>
    struct pred : std::bool_constant<requires(OBSERVER observer, STATE const& state) {
                      observer.notifyEntry(instanceValues(state).template get<T>());
                  } || requires(OBSERVER observer, STATE const& state) {
                      observer.notifyExit(instanceValues(state).template get<T>());
                  }> {};
};

template<typename OBSERVER, typename STATE>
inline constexpr bool values_notified_v =
    mtl::any_of_v<instance_types_t<STATE>, value_notified<OBSERVER, STATE>::template pred>;

// A state carrying T: as a static annotation or as an instance value
template<typename T>
struct carrying {
    template<typename STATE>
    struct pred : std::bool_constant<has_annotation_v<STATE, T> || has_value_v<STATE, T>> {};
};

} // namespace internal

namespace concepts {

// OBSERVER's observation of STATE reaches a notify hook: the annotation
// (or set element, or instance value) exists and a notifyEntry/notifyExit
// overload accepts it. These are the observing dispatch's own
// requires-expressions, so the concept cannot drift from what actually
// runs on an edge
template<typename OBSERVER, typename STATE>
concept notified_of =
    requires(OBSERVER observer) {
        observer.notifyEntry(OBSERVER::template annotation<STATE>());
    } ||
    requires(OBSERVER observer) {
        observer.notifyExit(OBSERVER::template annotation<STATE>());
    } || internal::set_notified_v<OBSERVER, STATE> || internal::values_notified_v<OBSERVER, STATE>;

} // namespace concepts

// Whether any state of TABLE carries the annotation T: an observer
// watching an annotation no state carries is wired to nothing (the
// check fsm::observing runs for the types an observer declares)
template<typename TABLE, typename T>
struct annotation_in_table
    : std::bool_constant<
          mtl::any_of_v<typename TABLE::states, internal::carrying<T>::template pred>> {};

template<typename TABLE, typename T>
inline constexpr bool annotation_in_table_v = annotation_in_table<TABLE, T>::value;

namespace internal {

template<typename TABLE>
struct carried_in {
    template<typename T>
    struct pred : annotation_in_table<TABLE, T> {};
};

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
// Equal-value edges are elided at compile time.
//
// A state's annotation set (fsm::annotate above) and its instance
// values (values(), fsm::annotate_ref) are observed without any observe
// declaration: every element type with a notifyEntry / notifyExit
// overload is delivered, in the set's order - the static set
// change-suppressed per element, the instance values on every edge:
//
//   struct phy_driver : fsm::observing<phy_driver> {
//       void notifyEntry(message_t const& message); // hand to hardware
//   };
//
// On one edge the order is: observe_static, the static set, the
// instance values. This ordering is part of the contract: a static
// annotation can prepare (e.g. reset) what the instance value then
// consumes.
//
// Both hook forms: where the edge is known (every exact edge, and the
// exit side of a wildcard) the edge form suppresses a value that does
// not change between the two states, at compile time; the entry side
// of a wildcard has no edge, so the machine takes the one-state form,
// which notifies every value of the state entered - the one place a
// value observer sees a re-notification.
//
// An observer watching an annotation no state carries is wired to
// nothing, silently (a renamed annotation, a typo in a type). An
// observer naming the annotation types it handles,
//
//   using observes = mtl::typelist<led_pattern, power>;
//
// gets each of them checked by every machine it is injected into: a
// listed type must be carried by a state of the table. Leave the
// declaration out for an observer that is handed to machines where it
// watches nothing by design. A derived class defining its own
// validate() hides the check; call observing<DERIVED>::validate<TABLE>()
// from it to keep it
template<typename DERIVED>
struct observing {

    template<typename STATE>
    static constexpr auto annotation()
        requires requires { DERIVED::template observe_static<STATE>(); }
    {
        return DERIVED::template observe_static<STATE>();
    }

    template<typename TABLE>
    static constexpr void validate()
    {
        if constexpr (requires { typename DERIVED::observes; }) {
            static_assert(mtl::all_of_v<typename DERIVED::observes,
                                        internal::carried_in<TABLE>::template pred>,
                          "fsm::observing: an annotation the observer declares to observe "
                          "is carried by no state of the table");
        }
    }

    // The edge form. The static path is per edge (the change check needs
    // both states); the instance values delegate to one body per valued
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
        this->template valuesExit<OLD_STATE>(machine);
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
        this->template valuesEnter<NEW_STATE>(machine);
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

    // The instance values, each element with a hook accepting it. The
    // machine is in STATE here, so getIf() cannot fail; the check is what
    // stops GCC reporting a potential null dereference inside <variant>
    // once this inlines at -Os
    template<typename STATE, typename MACHINE>
    void valuesExit(MACHINE& machine)
    {
        if constexpr (internal::values_notified_v<DERIVED, STATE>) {
            if (auto const* state = machine.template getIf<STATE>(); state != nullptr) {
                this->exitEach(internal::instanceValues(*state), internal::instance_types_t<STATE>{});
            }
        }
    }

    template<typename STATE, typename MACHINE>
    void valuesEnter(MACHINE& machine)
    {
        if constexpr (internal::values_notified_v<DERIVED, STATE>) {
            if (auto const* state = machine.template getIf<STATE>(); state != nullptr) {
                this->enterEach(internal::instanceValues(*state), internal::instance_types_t<STATE>{});
            }
        }
    }

    template<typename SET, typename... Ts>
    void exitEach(SET const& set, mtl::typelist<Ts...>)
    {
        auto& self = static_cast<DERIVED&>(*this);
        ([&] {
            if constexpr (requires { self.notifyExit(set.template get<Ts>()); }) {
                self.notifyExit(set.template get<Ts>());
            }
        }(), ...);
    }

    template<typename SET, typename... Ts>
    void enterEach(SET const& set, mtl::typelist<Ts...>)
    {
        auto& self = static_cast<DERIVED&>(*this);
        ([&] {
            if constexpr (requires { self.notifyEntry(set.template get<Ts>()); }) {
                self.notifyEntry(set.template get<Ts>());
            }
        }(), ...);
    }
};

} // namespace fsm
