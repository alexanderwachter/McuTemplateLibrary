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
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

namespace fsm {

namespace concepts {

// An event is the plain type a table names in on<>: a value, never a
// reference or a const one
template<typename T>
concept event = std::is_object_v<T> && std::same_as<T, std::remove_cv_t<T>>;

} // namespace concepts

// Matches every state in from<>; a state's own (state, event) group
// replaces the wildcard group, even when all its guards refuse
struct any_state {};

// Event injected by the timer policy when a state's timeout expires
struct timeout {};

// Event injected by the deadline policy when a phase deadline expires
// (fsm::deadlined, Timer.hpp); distinct from timeout so a state can react
// to both
struct deadline {};

// An event addressed to one machine: a composite state does not offer
// it to its submachine. Timer expiries are - a state's timer is its
// level's, and a timed sub-state (which always handles fsm::timeout)
// must not swallow its parent's. Specialize for own events with the
// same nature
template<concepts::event EVENT>
struct is_local_event : std::false_type {};

template<>
struct is_local_event<timeout> : std::true_type {};

template<>
struct is_local_event<deadline> : std::true_type {};

template<concepts::event EVENT>
inline constexpr bool local_event_v = is_local_event<EVENT>::value;

// A local event of a machine further down, on its way through the
// machines above: every event enters at the root, so a sub-state's
// expiry is decorated once per level it lies below the root, and each
// machine hands its submachine the event with one decoration less. No
// table names the decorated type - the machines on the way do not
// react to it themselves
template<concepts::event EVENT>
struct for_submachine {
    using event = EVENT;
};

namespace internal {

template<concepts::event EVENT, std::size_t LEVEL>
struct for_level : for_level<for_submachine<EVENT>, LEVEL - 1> {};

template<concepts::event EVENT>
struct for_level<EVENT, 0> : std::type_identity<EVENT> {};

// What a machine hands its submachine: the event as it is, or with
// one decoration less
template<concepts::event EVENT>
constexpr EVENT const& passedDown(EVENT const& event)
{
    return event;
}

template<concepts::event EVENT>
constexpr EVENT passedDown(for_submachine<EVENT> const&)
{
    return {};
}

} // namespace internal

// EVENT for the machine at nesting depth LEVEL (the root is 0), as it
// enters at the root: decorated LEVEL times
template<concepts::event EVENT, std::size_t LEVEL>
using for_level_t = typename internal::for_level<EVENT, LEVEL>::type;

namespace concepts {

// The machine owns one instance per context type and constructs it
// by itself
template<typename T>
concept context = std::default_initializable<T>;

} // namespace concepts

// The context types a state declares for itself (contexts) or a
// composite state for its submachine (parent_contexts)
template<concepts::context... CONTEXTs>
using contexts = mtl::typelist<CONTEXTs...>;

namespace internal {

// A state opts into machine-owned context by declaring the context
// types it is constructed with, in order:
//   using contexts = fsm::contexts<connection, negotiation>;
// Its constructors take (event, connection&, negotiation&) when built
// from an event and (connection&, negotiation&) otherwise, and keep
// the references under names of the state's own choosing
template<typename T>
concept context_holder =
    requires { typename T::contexts; } && mtl::concepts::typelist<typename T::contexts>;

template<typename STATE>
struct contexts_of : std::type_identity<mtl::typelist<>> {};

template<context_holder STATE>
struct contexts_of<STATE> : std::type_identity<typename STATE::contexts> {};

template<typename STATE>
using contexts_of_t = typename contexts_of<STATE>::type;

// Whether STATE is constructible from FRONT... followed by its contexts
template<typename STATE, typename CONTEXTS, typename... FRONT>
struct constructible_with;

template<typename STATE, typename... CONTEXTs, typename... FRONT>
struct constructible_with<STATE, mtl::typelist<CONTEXTs...>, FRONT...>
    : std::bool_constant<std::constructible_from<STATE, FRONT..., CONTEXTs&...>> {};

// ... from this event (and the contexts), or from the contexts alone
template<typename STATE, typename EVENT>
inline constexpr bool payload_constructible_v =
    constructible_with<STATE, contexts_of_t<STATE>, EVENT const&>::value;

template<typename STATE>
struct context_constructible : constructible_with<STATE, contexts_of_t<STATE>> {};

// Whether a machine's context tuple holds T as a reference into the
// parent machine (an inherited context) rather than as an own instance
template<typename T, typename CONTEXT_TUPLE>
struct holds_inherited_context;

template<typename T, typename... ELEMENTs>
struct holds_inherited_context<T, std::tuple<ELEMENTs...>>
    : std::disjunction<std::is_same<T&, ELEMENTs>...> {};

// The instance of context type T in a machine's context tuple: an own
// instance held by value, or the parent machine's behind the inherited
// reference
template<typename T, typename CONTEXT_TUPLE>
constexpr auto& contextOf(CONTEXT_TUPLE& contexts)
{
    if constexpr (holds_inherited_context<T, std::remove_cv_t<CONTEXT_TUPLE>>::value) {
        return std::get<T&>(contexts);
    } else {
        return std::get<T>(contexts);
    }
}

// Arguments constructing STATE in place inside a variant: the
// in_place tag and its contexts. The tuple round-trip through
// make_from_tuple is free: its prvalue is elided into the variant
// (measured GCC 15 -Os: direct stores, no tuple, no move)
template<typename STATE, typename CONTEXT_TUPLE>
constexpr auto initialArgs(CONTEXT_TUPLE& contexts)
{
    return [&]<typename... CONTEXTs>(mtl::typelist<CONTEXTs...>) {
        return std::forward_as_tuple(std::in_place_type<STATE>, contextOf<CONTEXTs>(contexts)...);
    }(contexts_of_t<STATE>{});
}

} // namespace internal

namespace concepts {

// States are classes; on entry they are constructed from the triggering
// event if such a constructor exists, default-constructed otherwise.
// Context states are constructed with their contexts instead.
template<typename T>
concept state = std::is_class_v<T> &&
                (std::default_initializable<T> || internal::context_holder<T>);

// A guard is a question, named by a tag: a class the machine
// constructs to pass to the injected object answering it, or one
// answering itself with a static check. Which of the two it is, the
// machine decides
template<typename T>
concept guard = std::is_class_v<T> && std::default_initializable<T>;

} // namespace concepts

template<concepts::state STATE>
struct from {};

template<concepts::event EVENT>
struct on {};

template<concepts::state STATE>
struct to {};

// A part of a row's condition that holds when GUARD does not
template<concepts::guard GUARD>
struct not_ {};

namespace internal {

// The guard behind a part of a condition
template<typename PART>
struct guard_of : std::type_identity<PART> {};

template<concepts::guard GUARD>
struct guard_of<fsm::not_<GUARD>> : std::type_identity<GUARD> {};

template<typename PART>
using guard_of_t = typename guard_of<PART>::type;

} // namespace internal

namespace concepts {

// A part of a row's condition: a guard, or not_<guard>
template<typename T>
concept guard_part = guard<internal::guard_of_t<T>>;

} // namespace concepts

// The row's condition: every part must hold, asked in order with
// short-circuit. A disjunction is another row of the same (state,
// event) pair
template<concepts::guard_part... PARTs>
struct guard {};

template<concepts::state STATE>
struct initial {};

// Marks a state the machine ends in: nothing leaves it, not even a
// from<any_state> transition. One entry per final state
template<concepts::state STATE>
struct final {};

namespace internal {

template<typename T>         struct is_from : std::false_type {};
template<concepts::state S>  struct is_from<fsm::from<S>> : std::true_type {};

template<typename T>         struct is_on : std::false_type {};
template<concepts::event E>  struct is_on<fsm::on<E>> : std::true_type {};

template<typename T>         struct is_to : std::false_type {};
template<concepts::state S>  struct is_to<fsm::to<S>> : std::true_type {};

template<typename T>                 struct is_guard : std::false_type {};
template<concepts::guard_part... Gs> struct is_guard<fsm::guard<Gs...>> : std::true_type {};

template<typename T>         struct is_initial : std::false_type {};
template<concepts::state S>  struct is_initial<fsm::initial<S>> : std::true_type {};

template<typename T>         struct is_final_role : std::false_type {};
template<concepts::state S>  struct is_final_role<fsm::final<S>> : std::true_type {};

template<typename T>         struct unwrap;
template<concepts::state S>  struct unwrap<fsm::from<S>>    { using type = S; };
template<concepts::event E>  struct unwrap<fsm::on<E>>      { using type = E; };
template<concepts::state S>  struct unwrap<fsm::to<S>>      { using type = S; };
template<concepts::state S>  struct unwrap<fsm::initial<S>> { using type = S; };
template<concepts::state S>  struct unwrap<fsm::final<S>>   { using type = S; };
template<>                   struct unwrap<mtl::nil_type>   { using type = mtl::nil_type; };

// Payload of the first role matching PREDICATE; nil_type if there is none
template<mtl::concepts::typelist LIST, template<typename> typename PREDICATE>
using find_role_t = typename unwrap<mtl::find_if_t<LIST, PREDICATE>>::type;

// The guard role's parts as a list, empty without the role
template<typename ROLE>              struct guard_parts { using type = mtl::typelist<>; };
template<concepts::guard_part... Gs> struct guard_parts<fsm::guard<Gs...>> { using type = mtl::typelist<Gs...>; };

template<mtl::concepts::typelist ROLES>
using guards_t = typename guard_parts<mtl::find_if_t<ROLES, is_guard>>::type;

// Whether a part's answer is inverted
template<concepts::guard_part PART> struct is_negated : std::false_type {};
template<concepts::guard GUARD>     struct is_negated<fsm::not_<GUARD>> : std::true_type {};

template<concepts::guard_part PART>
inline constexpr bool is_negated_v = is_negated<PART>::value;

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
concept transition_table_entry =
    transition<T> || internal::is_initial<T>::value || internal::is_final_role<T>::value;

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
};

namespace internal {

template<concepts::transition TRANSITION>
inline constexpr bool is_internal_v = std::is_same_v<typename TRANSITION::to, internal_target>;

template<typename T>
struct is_internal_target : std::is_same<T, internal_target> {};

} // namespace internal

} // namespace fsm
