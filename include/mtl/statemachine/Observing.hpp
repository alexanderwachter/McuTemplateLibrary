/*
 * fsm: value observation - annotation sets and the fsm::observing base
 * with compile-time change suppression
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <mtl/statemachine/Table.hpp>
#include <mtl/TypelistAlgorithms.hpp>
#include <mtl/Typelist.hpp>

#include <concepts>
#include <tuple>
#include <type_traits>
#include <utility>

namespace fsm {

namespace concepts {

// An annotation is keyed by its plain type: a value, never a
// reference or a const one
template<typename T>
concept annotation = std::is_object_v<T> && std::same_as<T, std::remove_cv_t<T>>;

} // namespace concepts

namespace internal {

template<concepts::observer OBSERVER, concepts::state STATE>
inline constexpr bool observes_v = requires { OBSERVER::template annotation<STATE>(); };

// Whether STATE's annotation differs from OTHER's: an annotation OTHER
// lacks, or of another type, always counts as a change; an unobserved
// STATE never notifies
template<concepts::observer OBSERVER, concepts::state STATE, concepts::state OTHER>
struct annotation_changes : std::bool_constant<observes_v<OBSERVER, STATE>> {};

template<concepts::observer OBSERVER, concepts::state STATE, concepts::state OTHER>
    requires observes_v<OBSERVER, STATE> && observes_v<OBSERVER, OTHER> &&
             std::same_as<decltype(OBSERVER::template annotation<STATE>()),
                          decltype(OBSERVER::template annotation<OTHER>())>
struct annotation_changes<OBSERVER, STATE, OTHER>
    : std::bool_constant<OBSERVER::template annotation<STATE>() !=
                         OBSERVER::template annotation<OTHER>()> {};

template<concepts::observer OBSERVER, concepts::state STATE, concepts::state OTHER>
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
concept annotated = concepts::state<STATE> && requires {
    STATE::annotations;
    requires is_annotation_set<std::remove_cvref_t<decltype(STATE::annotations)>>::value;
};

// A state with per-instance values; values() may return a set or one
// value, which instanceValues() wraps into a set (by reference when
// values() returns a reference)
template<typename STATE>
concept valued = concepts::state<STATE> && requires(STATE const& state) { state.values(); };

template<valued STATE>
constexpr auto instanceValues(STATE const& state)
{
    if constexpr (is_annotation_set<std::remove_cvref_t<decltype(state.values())>>::value) {
        return state.values();
    } else {
        return annotate_ref(state.values());
    }
}

template<concepts::state STATE>
struct instance_types : std::type_identity<mtl::typelist<>> {};

template<valued STATE>
struct instance_types<STATE>
    : std::type_identity<typename decltype(instanceValues(std::declval<STATE const&>()))::types> {};

template<concepts::state STATE>
using instance_types_t = typename instance_types<STATE>::type;

template<concepts::state STATE, typename T>
inline constexpr bool has_value_v = mtl::has_a_v<instance_types_t<STATE>, std::remove_cvref_t<T>>;

// The element types of a state's set, none for a state without one
template<concepts::state STATE>
struct annotation_types : std::type_identity<mtl::typelist<>> {};

template<annotated STATE>
struct annotation_types<STATE>
    : std::type_identity<typename std::remove_cvref_t<decltype(STATE::annotations)>::types> {};

template<concepts::state STATE>
using annotation_types_t = typename annotation_types<STATE>::type;

// Lazy: the set is only named for annotated states (a plain && would
// substitute both operands)
template<concepts::state STATE, typename T>
struct has_annotation : std::false_type {};

template<annotated STATE, typename T>
struct has_annotation<STATE, T>
    : std::bool_constant<std::remove_cvref_t<decltype(STATE::annotations)>::template has<T>> {};

template<concepts::state STATE, typename T>
inline constexpr bool has_annotation_v = has_annotation<STATE, T>::value;

// Whether STATE's T element differs from OTHER's: an element OTHER lacks
// always counts as a change, a state without the element never notifies
template<concepts::annotation T, concepts::state STATE, concepts::state OTHER>
struct set_annotation_changes : std::bool_constant<has_annotation_v<STATE, T>> {};

template<concepts::annotation T, concepts::state STATE, concepts::state OTHER>
    requires has_annotation_v<STATE, T> && has_annotation_v<OTHER, T>
struct set_annotation_changes<T, STATE, OTHER>
    : std::bool_constant<STATE::annotations.template get<T>() !=
                         OTHER::annotations.template get<T>()> {};

template<concepts::annotation T, concepts::state STATE, concepts::state OTHER>
inline constexpr bool set_annotation_changes_v = set_annotation_changes<T, STATE, OTHER>::value;

// Whether STATE's T element reaches one of OBSERVER's hooks
template<concepts::observer OBSERVER, concepts::state STATE>
struct element_notified {
    template<concepts::annotation T>
    struct pred : std::bool_constant<requires(OBSERVER observer) {
                      observer.notifyEntry(STATE::annotations.template get<T>());
                  } || requires(OBSERVER observer) {
                      observer.notifyExit(STATE::annotations.template get<T>());
                  }> {};
};

template<concepts::observer OBSERVER, concepts::state STATE>
inline constexpr bool set_notified_v =
    mtl::any_of_v<annotation_types_t<STATE>, element_notified<OBSERVER, STATE>::template pred>;

// Whether STATE's T value reaches one of OBSERVER's hooks
template<concepts::observer OBSERVER, concepts::state STATE>
struct value_notified {
    template<concepts::annotation T>
    struct pred : std::bool_constant<requires(OBSERVER observer, STATE const& state) {
                      observer.notifyEntry(instanceValues(state).template get<T>());
                  } || requires(OBSERVER observer, STATE const& state) {
                      observer.notifyExit(instanceValues(state).template get<T>());
                  }> {};
};

template<concepts::observer OBSERVER, concepts::state STATE>
inline constexpr bool values_notified_v =
    mtl::any_of_v<instance_types_t<STATE>, value_notified<OBSERVER, STATE>::template pred>;

// The set elements of STATE that change against OTHER and reach
// OBSERVER's exit / entry hook: what an edge delivers, as a list
template<concepts::observer OBSERVER, concepts::state STATE, concepts::state OTHER>
struct exit_delivers {
    template<concepts::annotation T>
    struct pred : std::bool_constant<set_annotation_changes_v<T, STATE, OTHER> &&
                                     requires(OBSERVER observer) {
                                         observer.notifyExit(STATE::annotations.template get<T>());
                                     }> {};
};

template<concepts::observer OBSERVER, concepts::state STATE, concepts::state OTHER>
struct entry_delivers {
    template<concepts::annotation T>
    struct pred : std::bool_constant<set_annotation_changes_v<T, STATE, OTHER> &&
                                     requires(OBSERVER observer) {
                                         observer.notifyEntry(STATE::annotations.template get<T>());
                                     }> {};
};

template<concepts::observer OBSERVER, concepts::state STATE, concepts::state OTHER>
using exit_delivered_t =
    mtl::filter_t<annotation_types_t<STATE>, exit_delivers<OBSERVER, STATE, OTHER>::template pred>;

template<concepts::observer OBSERVER, concepts::state STATE, concepts::state OTHER>
using entry_delivered_t =
    mtl::filter_t<annotation_types_t<STATE>, entry_delivers<OBSERVER, STATE, OTHER>::template pred>;

// The instance values of STATE with an exit / entry hook accepting them
template<concepts::observer OBSERVER, concepts::state STATE>
struct value_exit_delivers {
    template<concepts::annotation T>
    struct pred : std::bool_constant<requires(OBSERVER observer, STATE const& state) {
                      observer.notifyExit(instanceValues(state).template get<T>());
                  }> {};
};

template<concepts::observer OBSERVER, concepts::state STATE>
struct value_entry_delivers {
    template<concepts::annotation T>
    struct pred : std::bool_constant<requires(OBSERVER observer, STATE const& state) {
                      observer.notifyEntry(instanceValues(state).template get<T>());
                  }> {};
};

template<concepts::observer OBSERVER, concepts::state STATE>
using value_exit_delivered_t =
    mtl::filter_t<instance_types_t<STATE>, value_exit_delivers<OBSERVER, STATE>::template pred>;

template<concepts::observer OBSERVER, concepts::state STATE>
using value_entry_delivered_t =
    mtl::filter_t<instance_types_t<STATE>, value_entry_delivers<OBSERVER, STATE>::template pred>;

// A state carrying T: as a static annotation or as an instance value
template<concepts::annotation T>
struct carrying {
    template<concepts::state STATE>
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

// Whether any state of TABLE - at any nesting level - carries the
// annotation T: an observer watching an annotation no state carries is
// wired to nothing (the check fsm::observing runs for the types an
// observer declares)
template<concepts::transition_table TABLE, concepts::annotation T>
struct annotation_in_table
    : std::bool_constant<
          mtl::any_of_v<all_states_t<TABLE>, internal::carrying<T>::template pred>> {};

template<concepts::transition_table TABLE, concepts::annotation T>
inline constexpr bool annotation_in_table_v = annotation_in_table<TABLE, T>::value;

namespace internal {

template<concepts::transition_table TABLE>
struct carried_in {
    template<concepts::annotation T>
    struct pred : annotation_in_table<TABLE, T> {};
};

} // namespace internal

// Value observer base: the derived class names the watched member once and
// provides notifyEntry(value) (new state's value) and/or notifyExit(value)
// (old state's value, old state still alive), each optional:
//
//   struct lamp_driver : fsm::observing<lamp_driver> {
//       template<concepts::state STATE>
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
//   using observes = fsm::annotations<led_pattern, power>;
//
// gets each of them checked by every machine it is injected into: a
// listed type must be carried by a state of the table. Leave the
// declaration out for an observer that is handed to machines where it
// watches nothing by design. A derived class defining its own
// validate() hides the check; call observing<DERIVED>::validate<TABLE>()
// from it to keep it
template<typename DERIVED>
struct observing {

    template<concepts::state STATE>
    static constexpr auto annotation()
        requires requires { DERIVED::template observe_static<STATE>(); }
    {
        return DERIVED::template observe_static<STATE>();
    }

    template<concepts::transition_table TABLE>
    static constexpr void validate()
    {
        if constexpr (requires { typename DERIVED::observes; }) {
            static_assert(mtl::all_of_v<typename DERIVED::observes,
                                        internal::carried_in<TABLE>::template pred>,
                          "fsm::observing: an annotation the observer declares to observe "
                          "is carried by no state of the table");
        }
    }

    // Whether anything of STATE reaches an exit / entry hook of DERIVED.
    // The hooks exist for such states only, so a state this observer
    // ignores costs the machine no function on any of its edges. Both
    // forms go together: the machine takes the one-state form where the
    // edge form is missing, which would re-notify an unchanged value
    template<concepts::state STATE>
    static constexpr bool exits_notified =
        requires(DERIVED observer) {
            observer.notifyExit(DERIVED::template annotation<STATE>());
        } || !mtl::empty_v<internal::exit_delivered_t<DERIVED, STATE, mtl::nil_type>> ||
        !mtl::empty_v<internal::value_exit_delivered_t<DERIVED, STATE>>;

    template<concepts::state STATE>
    static constexpr bool entries_notified =
        requires(DERIVED observer) {
            observer.notifyEntry(DERIVED::template annotation<STATE>());
        } || !mtl::empty_v<internal::entry_delivered_t<DERIVED, STATE, mtl::nil_type>> ||
        !mtl::empty_v<internal::value_entry_delivered_t<DERIVED, STATE>>;

    // The edge form. The static path is per edge (the change check needs
    // both states); the instance values delegate to one body per valued
    // state
    template<concepts::state OLD_STATE, concepts::state NEW_STATE, typename MACHINE>
        requires (exits_notified<OLD_STATE>)
    void onExitFrom(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (internal::annotation_changes_v<DERIVED, OLD_STATE, NEW_STATE>) {
            if constexpr (requires { self.notifyExit(DERIVED::template annotation<OLD_STATE>()); }) {
                self.notifyExit(DERIVED::template annotation<OLD_STATE>());
            }
        }
        setExit<OLD_STATE>(
            internal::exit_delivered_t<DERIVED, OLD_STATE, NEW_STATE>{});
        valuesExit<OLD_STATE>(machine);
    }

    template<concepts::state OLD_STATE, concepts::state NEW_STATE, typename MACHINE>
        requires (entries_notified<NEW_STATE>)
    void onEnterFrom(MACHINE& machine)
    {
        auto& self = static_cast<DERIVED&>(*this);
        if constexpr (internal::annotation_changes_v<DERIVED, NEW_STATE, OLD_STATE>) {
            if constexpr (requires { self.notifyEntry(DERIVED::template annotation<NEW_STATE>()); }) {
                self.notifyEntry(DERIVED::template annotation<NEW_STATE>());
            }
        }
        setEnter<NEW_STATE>(
            internal::entry_delivered_t<DERIVED, NEW_STATE, OLD_STATE>{});
        valuesEnter<NEW_STATE>(machine);
    }

    // The one-state form: no other state to compare against, which is the
    // edge form against mtl::nil_type - every value counts as a change
    template<concepts::state STATE, typename MACHINE>
        requires (exits_notified<STATE>)
    void onExit(MACHINE& machine)
    {
        onExitFrom<STATE, mtl::nil_type>(machine);
    }

    template<concepts::state STATE, typename MACHINE>
        requires (entries_notified<STATE>)
    void onEnter(MACHINE& machine)
    {
        onEnterFrom<mtl::nil_type, STATE>(machine);
    }

protected:
    // A mixin: only ever a base of DERIVED. The derived observer is
    // therefore no aggregate for the outside - give it a constructor
    // rather than initializing its members with braces
    observing() = default;

private:
    // The set elements the edge delivers, already selected
    template<concepts::state STATE, concepts::annotation... Ts>
    void setExit(mtl::typelist<Ts...>)
    {
        auto& self = static_cast<DERIVED&>(*this);
        (self.notifyExit(STATE::annotations.template get<Ts>()), ...);
    }

    template<concepts::state STATE, concepts::annotation... Ts>
    void setEnter(mtl::typelist<Ts...>)
    {
        auto& self = static_cast<DERIVED&>(*this);
        (self.notifyEntry(STATE::annotations.template get<Ts>()), ...);
    }

    // The instance values with a hook accepting them. The machine is in
    // STATE here, so getIf() cannot fail; the check is what stops GCC
    // reporting a potential null dereference inside <variant> once this
    // inlines at -Os
    template<concepts::state STATE, typename MACHINE>
    void valuesExit(MACHINE& machine)
    {
        using delivered = internal::value_exit_delivered_t<DERIVED, STATE>;
        if constexpr (!mtl::empty_v<delivered>) {
            if (auto const* state = machine.template getIf<STATE>(); state != nullptr) {
                exitEach(internal::instanceValues(*state), delivered{});
            }
        }
    }

    template<concepts::state STATE, typename MACHINE>
    void valuesEnter(MACHINE& machine)
    {
        using delivered = internal::value_entry_delivered_t<DERIVED, STATE>;
        if constexpr (!mtl::empty_v<delivered>) {
            if (auto const* state = machine.template getIf<STATE>(); state != nullptr) {
                enterEach(internal::instanceValues(*state), delivered{});
            }
        }
    }

    template<typename SET, concepts::annotation... Ts>
    void exitEach(SET const& set, mtl::typelist<Ts...>)
    {
        auto& self = static_cast<DERIVED&>(*this);
        (self.notifyExit(set.template get<Ts>()), ...);
    }

    template<typename SET, concepts::annotation... Ts>
    void enterEach(SET const& set, mtl::typelist<Ts...>)
    {
        auto& self = static_cast<DERIVED&>(*this);
        (self.notifyEntry(set.template get<Ts>()), ...);
    }
};

} // namespace fsm
