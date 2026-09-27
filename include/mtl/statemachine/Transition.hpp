/*
 * fsm: states, the transition roles and the transition types.
 * The contract lives in <mtl/StateMachine.hpp>
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

// Matches every state in from<>; a state's own (state, event) group
// replaces the wildcard group, even when all its guards refuse
struct any_state {};

// Event injected by the timer policy when a state's timeout expires
struct timeout {};

// Event injected by the deadline policy when a phase deadline expires
// (fsm::deadlined, Timer.hpp); distinct from timeout so a state can react
// to both
struct deadline {};

namespace internal {

// A state opts into machine-owned context by holding a reference
// member named context, initialized by its constructors
template<typename T>
concept context_holder = requires { T::context; } && std::is_reference_v<decltype(T::context)>;

template<typename STATE>
struct has_context : std::bool_constant<context_holder<STATE>> {};

template<context_holder STATE>
struct context_of : std::type_identity<std::remove_reference_t<decltype(STATE::context)>> {};

template<context_holder STATE>
using context_of_t = typename context_of<STATE>::type;

// Whether STATE is constructed from this event (with its context when
// it has one)
template<typename STATE, typename EVENT>
struct payload_constructible : std::bool_constant<std::constructible_from<STATE, EVENT const&>> {};

template<context_holder STATE, typename EVENT>
struct payload_constructible<STATE, EVENT>
    : std::bool_constant<std::constructible_from<STATE, EVENT const&, context_of_t<STATE>&>> {};

template<typename STATE, typename EVENT>
inline constexpr bool payload_constructible_v = payload_constructible<STATE, EVENT>::value;

// Arguments constructing STATE in place inside a variant. The tuple
// round-trip through make_from_tuple is free: its prvalue is elided
// into the variant (measured GCC 15 -Os: direct stores, no tuple, no
// move)
template<typename STATE, typename CONTEXT_TUPLE>
constexpr auto initialArgs(CONTEXT_TUPLE& contexts)
{
    if constexpr (context_holder<STATE>) {
        return std::forward_as_tuple(std::in_place_type<STATE>,
                                     std::get<context_of_t<STATE>>(contexts));
    } else {
        return std::make_tuple(std::in_place_type<STATE>);
    }
}

} // namespace internal

namespace concepts {

// States are classes; on entry they are constructed from the triggering
// event if such a constructor exists, default-constructed otherwise.
// Context states are constructed with their context instead.
template<typename T>
concept state = std::is_class_v<T> &&
                (std::default_initializable<T> || internal::context_holder<T>);

} // namespace concepts

template<concepts::state STATE>
struct from {};

template<typename EVENT>
struct on {};

template<concepts::state STATE>
struct to {};

// The row's condition: every part must hold, asked in order with
// short-circuit; a part not_<G> holds when G does not. A disjunction
// is another row of the same (state, event) pair
template<typename... GUARDs>
struct guard {};

template<typename GUARD>
struct not_ {};

template<concepts::state STATE>
struct initial {};

namespace internal {

template<typename T> struct is_from : std::false_type {};
template<typename S> struct is_from<fsm::from<S>> : std::true_type {};

template<typename T> struct is_on : std::false_type {};
template<typename E> struct is_on<fsm::on<E>> : std::true_type {};

template<typename T> struct is_to : std::false_type {};
template<typename S> struct is_to<fsm::to<S>> : std::true_type {};

template<typename T>     struct is_guard : std::false_type {};
template<typename... Gs> struct is_guard<fsm::guard<Gs...>> : std::true_type {};

template<typename T> struct is_initial : std::false_type {};
template<typename S> struct is_initial<fsm::initial<S>> : std::true_type {};

template<typename T> struct unwrap;
template<typename S> struct unwrap<fsm::from<S>>    { using type = S; };
template<typename E> struct unwrap<fsm::on<E>>      { using type = E; };
template<typename S> struct unwrap<fsm::to<S>>      { using type = S; };
template<typename S> struct unwrap<fsm::initial<S>> { using type = S; };
template<>           struct unwrap<mtl::nil_type>   { using type = mtl::nil_type; };

// Payload of the first role matching PREDICATE; nil_type if there is none
template<mtl::concepts::typelist LIST, template<typename> typename PREDICATE>
using find_role_t = typename unwrap<mtl::find_if_t<LIST, PREDICATE>>::type;

// The guard role's parts as a list, empty without the role
template<typename ROLE>  struct guard_parts { using type = mtl::typelist<>; };
template<typename... Gs> struct guard_parts<fsm::guard<Gs...>> { using type = mtl::typelist<Gs...>; };

template<mtl::concepts::typelist ROLES>
using guards_t = typename guard_parts<mtl::find_if_t<ROLES, is_guard>>::type;

// A part is a guard, or not_<guard>: the guard behind it, and whether
// its answer is inverted
template<typename PART> struct guard_of : std::type_identity<PART> {};
template<typename G>    struct guard_of<fsm::not_<G>> : std::type_identity<G> {};

template<typename PART>
using guard_of_t = typename guard_of<PART>::type;

template<typename PART> struct is_negated : std::false_type {};
template<typename G>    struct is_negated<fsm::not_<G>> : std::true_type {};

template<typename PART>
inline constexpr bool is_negated_v = is_negated<PART>::value;

// A guard is a tag: an empty class the machine passes to the injected
// object answering it, or a class answering itself with a static
// check. Which of the two it is, the machine decides
template<typename PART>
struct is_guard_tag : std::bool_constant<std::is_class_v<guard_of_t<PART>> &&
                                         std::default_initializable<guard_of_t<PART>>> {};

// Stand-in for any event in unevaluated contexts: validates a guard's
// two-argument (state, event) form when the event type is unknown
struct any_payload {
    template<typename T>
    operator T const&() const;
};

} // namespace internal

namespace concepts {

// The guard contract is guard_for below, validated against the
// transition's from-state; there is no state-independent guard concept
// because a templated check (a guard shared by several states via
// check(auto const&)) cannot be probed by name alone

// check(from_state, event) for conditions on the event payload before
// any handler applied it, check(from_state) for conditions on state
// data, check() for state-independent ones. A static guard answers
// itself:
template<typename GUARD, typename STATE, typename EVENT>
concept event_guard_for = requires(STATE const& state, EVENT const& event) {
    { GUARD::check(state, event) } -> std::convertible_to<bool>;
};

template<typename GUARD, typename STATE>
concept state_guard_for = requires(STATE const& state) {
    { GUARD::check(state) } -> std::convertible_to<bool>;
};

template<typename GUARD>
concept stateless_guard = requires {
    { GUARD::check() } -> std::convertible_to<bool>;
};

// The event form is validated with a payload stand-in - the concrete
// event type is only known at the process() call
template<typename GUARD, typename STATE>
concept guard_for = state_guard_for<GUARD, STATE> ||
                    event_guard_for<GUARD, STATE, internal::any_payload> ||
                    stateless_guard<GUARD>;

// An object injected into the machine answers the guard GUARD - the
// table's question, a tag - with the same three forms, the tag selecting
// the overload: check(GUARD, from_state, event), check(GUARD,
// from_state), check(GUARD). An injected answer wins over a static one
template<typename OBJECT, typename GUARD, typename STATE, typename EVENT>
concept answers_event_guard =
    requires(OBJECT& object, STATE const& state, EVENT const& event) {
        { object.check(GUARD{}, state, event) } -> std::convertible_to<bool>;
    };

template<typename OBJECT, typename GUARD, typename STATE>
concept answers_state_guard = requires(OBJECT& object, STATE const& state) {
    { object.check(GUARD{}, state) } -> std::convertible_to<bool>;
};

template<typename OBJECT, typename GUARD>
concept answers_stateless_guard = requires(OBJECT& object) {
    { object.check(GUARD{}) } -> std::convertible_to<bool>;
};

template<typename OBJECT, typename GUARD, typename STATE>
concept answers_guard_for = answers_state_guard<OBJECT, GUARD, STATE> ||
                            answers_event_guard<OBJECT, GUARD, STATE, internal::any_payload> ||
                            answers_stateless_guard<OBJECT, GUARD>;

// Anything exposing the four role aliases works as a transition
template<typename T>
concept transition = requires {
    typename T::from;
    typename T::event;
    typename T::to;
    typename T::guards;
};

template<typename T>
concept transition_table_entry = transition<T> || internal::is_initial<T>::value;

template<typename T>
concept transition_role = internal::is_from<T>::value || internal::is_on<T>::value ||
                          internal::is_to<T>::value || internal::is_guard<T>::value;

} // namespace concepts

// The named arguments may appear in any order
template<concepts::transition_role... ROLEs>
struct transition {
private:
    using roles = mtl::typelist<ROLEs...>;
    static_assert(mtl::count_if_v<roles, internal::is_from> == 1,
                  "transition: exactly one from<STATE> required");
    static_assert(mtl::count_if_v<roles, internal::is_on> == 1,
                  "transition: exactly one on<EVENT> required");
    static_assert(mtl::count_if_v<roles, internal::is_to> == 1,
                  "transition: exactly one to<STATE> required");
    static_assert(mtl::count_if_v<roles, internal::is_guard> <= 1,
                  "transition: at most one guard<GUARDs...> allowed - list the parts in it");

public:
    using from   = internal::find_role_t<roles, internal::is_from>;
    using event  = internal::find_role_t<roles, internal::is_on>;
    using to     = internal::find_role_t<roles, internal::is_to>;
    using guards = internal::guards_t<roles>; // the condition's parts, empty if unguarded

private:
    static_assert(mtl::all_of_v<guards, internal::is_guard_tag>,
                  "transition: a guard must be a default-constructible class - a tag an "
                  "injected object answers, or a static check of its own - or not_<one>");
};

// The to-alias of internal transitions: never a state of the table
struct internal_target {};

// Handles the event inside the from-state instead of transitioning
template<concepts::transition_role... ROLEs>
struct internal_transition {
private:
    using roles = mtl::typelist<ROLEs...>;
    static_assert(mtl::count_if_v<roles, internal::is_from> == 1,
                  "internal_transition: exactly one from<STATE> required");
    static_assert(mtl::count_if_v<roles, internal::is_on> == 1,
                  "internal_transition: exactly one on<EVENT> required");
    static_assert(mtl::count_if_v<roles, internal::is_to> == 0,
                  "internal_transition: to<STATE> is not allowed");
    static_assert(mtl::count_if_v<roles, internal::is_guard> <= 1,
                  "internal_transition: at most one guard<GUARDs...> allowed - list the parts "
                  "in it");

public:
    using from   = internal::find_role_t<roles, internal::is_from>;
    using event  = internal::find_role_t<roles, internal::is_on>;
    using to     = internal_target;
    using guards = internal::guards_t<roles>; // the condition's parts, empty if unguarded

private:
    static_assert(!std::is_same_v<from, any_state>,
                  "internal_transition: from<any_state> is not supported");
    static_assert(mtl::all_of_v<guards, internal::is_guard_tag>,
                  "internal_transition: a guard must be a default-constructible class - a tag "
                  "an injected object answers, or a static check of its own - or not_<one>");
};

namespace internal {

template<typename TRANSITION>
inline constexpr bool is_internal_v = std::is_same_v<typename TRANSITION::to, internal_target>;

template<typename T>
struct is_internal_target : std::is_same<T, internal_target> {};

} // namespace internal

} // namespace fsm
