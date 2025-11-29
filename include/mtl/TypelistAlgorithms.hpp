/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Typelist.hpp"
#include <type_traits>

namespace mtl {

// is the true type if for all of the elements the predicate has a value of true
template<concepts::typelist LIST,  template<typename> typename PREDICATE>
struct all_of;

template<concepts::typelist LIST, template<typename> typename PREDICATE>
inline constexpr bool all_of_v = all_of<LIST, PREDICATE>::value;

template<template<typename> typename PREDICATE, typename... ELEMENTs>
struct all_of<typelist<ELEMENTs...>, PREDICATE> : std::conjunction<PREDICATE<ELEMENTs>...> {};

// is the true type if for any of the elements the predicate has a value of true
template<concepts::typelist LIST,  template<typename> typename PREDICATE>
struct any_of;

template<concepts::typelist LIST, template<typename> typename PREDICATE>
inline constexpr bool any_of_v = any_of<LIST, PREDICATE>::value;

template<template<typename> typename PREDICATE, typename... ELEMENTs>
struct any_of<typelist<ELEMENTs...>, PREDICATE> : std::disjunction<PREDICATE<ELEMENTs>...> {};

// is the true type if for any of the elements the predicate has a value of true
template<concepts::typelist LIST,  template<typename> typename PREDICATE>
struct none_of;

template<concepts::typelist LIST, template<typename> typename PREDICATE>
inline constexpr bool none_of_v = none_of<LIST, PREDICATE>::value;

template<template<typename> typename PREDICATE, typename... ELEMENTs>
struct none_of<typelist<ELEMENTs...>, PREDICATE> : std::conjunction<std::negation<PREDICATE<ELEMENTs>>...> {};


// The list contains at least on of T
template<concepts::typelist LIST, typename T>
struct has_a;

template<concepts::typelist LIST, typename T>
inline constexpr bool has_a_v = has_a<LIST, T>::value;

template<typename T, typename... ELEMENTs>
struct has_a<typelist<ELEMENTs...>, T> : std::disjunction<std::is_same<T, ELEMENTs>...> {};

// Sum of all values of the types in the list
template<concepts::typelist LIST>
struct sum;

template<concepts::typelist LIST>
inline constexpr auto sum_v = sum<LIST>::value;

template<>
struct sum<typelist<>> : std::integral_constant<int, 0> {};

template<typename... ELEMENTs>
struct sum<typelist<ELEMENTs...>> : std::integral_constant<std::common_type_t<decltype(ELEMENTs::value)...>, (ELEMENTs::value + ...)> {};

// type is the element of the typelist where the predicate matches first. If no element matches, type is the nil_type
template<concepts::typelist LIST,  template<typename> typename PREDICATE>
struct find_if;

template<concepts::typelist LIST, template<typename> typename PREDICATE>
using find_if_t = typename find_if<LIST, PREDICATE>::type;

template<template<typename> typename PREDICATE>
struct find_if<typelist<>, PREDICATE>
{
    using type = nil_type;
};

template<template<typename> typename PREDICATE, typename FIRST, typename... RESTs>
struct find_if<typelist<FIRST, RESTs...>, PREDICATE>
{
    using type = std::conditional_t<PREDICATE<FIRST>::value, FIRST, find_if_t<typelist<RESTs...>, PREDICATE>>;
};

namespace internal
{

template<template<typename, typename> typename COMPARE, concepts::typelist LIST>
struct min_max;

template<template<typename, typename> typename COMPARE, concepts::typelist LIST>
using min_max_t = typename min_max<COMPARE, LIST>::type;


template<template<typename, typename> typename COMPARE, typename FIRST>
struct min_max<COMPARE, typelist<FIRST>>
{
    using type = FIRST;
};

template<template<typename, typename> typename COMPARE, typename FIRST, typename SECOND, typename... RESTs>
struct min_max<COMPARE, typelist<FIRST, SECOND, RESTs...>>
{
    using type = std::conditional_t<COMPARE<FIRST, SECOND>::value, min_max_t<COMPARE, typelist<FIRST, RESTs...>>, min_max_t<COMPARE, typelist<SECOND, RESTs...>>>;
};

template<typename T1, typename T2>
struct gt_value
{
    static constexpr bool value = T1::value > T2::value;
};

template<typename T1, typename T2>
struct lt_value 
{
    static constexpr bool value = T1::value < T2::value;
};

} // namespace internal

// type is the element that is satisfies compare over all other
template<concepts::typelist LIST,  template<typename, typename> typename COMPARE = internal::lt_value>
struct min
{
    using type = internal::min_max_t<COMPARE, LIST>;
};

template<concepts::typelist LIST,  template<typename, typename> typename COMPARE = internal::lt_value>
using min_t = typename min<LIST, COMPARE>::type;


// type is the element that is satisfies compare over all other
template<concepts::typelist LIST,  template<typename, typename> typename COMPARE = internal::gt_value>
struct max
{
    using type = internal::min_max_t<COMPARE, LIST>;
};

template<concepts::typelist LIST,  template<typename, typename> typename COMPARE = internal::gt_value>
using max_t = typename max<LIST, COMPARE>::type;


// type is a typelist with the elements of the list that satisfies the predicate
template<concepts::typelist LIST,  template<typename> typename PREDICATE>
struct filter;

template<concepts::typelist LIST, template<typename> typename PREDICATE>
using filter_t = typename filter<LIST, PREDICATE>::type;

template<template<typename> typename PREDICATE>
struct filter<typelist<>, PREDICATE> : std::type_identity<typelist<>> {};

template<template<typename> typename PREDICATE, typename FIRST, typename... RESTs>
struct filter<typelist<FIRST, RESTs...>, PREDICATE>
{
    using type = std::conditional_t<PREDICATE<FIRST>::value, prepend_t<FIRST, filter_t<typelist<RESTs...>, PREDICATE>>, filter_t<typelist<RESTs...>, PREDICATE>>;
};


// type is a typelist where the elements that satisfies the predicate are removed
template<concepts::typelist LIST,  template<typename> typename PREDICATE>
struct remove_if;

template<concepts::typelist LIST, template<typename> typename PREDICATE>
using remove_if_t = typename remove_if<LIST, PREDICATE>::type;

template<template<typename> typename PREDICATE>
struct remove_if<typelist<>, PREDICATE> : std::type_identity<typelist<>> {};

template<template<typename> typename PREDICATE, typename FIRST, typename... RESTs>
struct remove_if<typelist<FIRST, RESTs...>, PREDICATE>
{
    using type = std::conditional_t<PREDICATE<FIRST>::value, remove_if_t<typelist<RESTs...>, PREDICATE>, prepend_t<FIRST, remove_if_t<typelist<RESTs...>, PREDICATE>>>;
};


// removes the firs occurrences of a type if there exist the same type later in the list
template<concepts::typelist LIST>
struct unique_keep_last;

template<concepts::typelist LIST>
using unique_keep_last_t = typename unique_keep_last<LIST>::type;

template<>
struct unique_keep_last<typelist<>> : std::type_identity<typelist<>> {};

template<typename FIRST, typename... RESTs>
struct unique_keep_last<typelist<FIRST, RESTs...>>
{
    using type = std::conditional_t<has_a_v<typelist<RESTs...>, FIRST>, unique_keep_last_t<typelist<RESTs...>>, prepend_t<FIRST, unique_keep_last_t<typelist<RESTs...>>>>;
};

// keeps the first occurrence of a type in the list and removes all further
template<concepts::typelist LIST>
struct unique : reverse<unique_keep_last_t<reverse_t<LIST>>> {};

template<concepts::typelist LIST>
using unique_t = typename unique<LIST>::type;


// count the number of elements that satisfies the predicate
template<concepts::typelist LIST,  template<typename> typename PREDICATE>
struct count_if;

template<concepts::typelist LIST, template<typename> typename PREDICATE>
inline constexpr std::size_t count_if_v = count_if<LIST, PREDICATE>::value;

template<template<typename> typename PREDICATE>
struct count_if<typelist<>, PREDICATE> : std::integral_constant<std::size_t, 0> {};

template<template<typename> typename PREDICATE, typename FIRST, typename... RESTs>
struct count_if<typelist<FIRST, RESTs...>, PREDICATE> : std::integral_constant<std::size_t, count_if_v<typelist<RESTs...>, PREDICATE> + (PREDICATE<FIRST>::value ? 1U : 0U)> {};

// Apply the operation on each element and make a new list from the operation::type
template<concepts::typelist LIST, template<typename> typename OPERATION>
struct transform;

template<concepts::typelist LIST, template<typename> typename OPERATION>
using transform_t = typename transform<LIST, OPERATION>::type;

template<template<typename> typename OPERATION, typename... ELEMENTs>
struct transform<typelist<ELEMENTs...>, OPERATION>
{
    using type = typelist<typename OPERATION<ELEMENTs>::type...>;
};


namespace internal {

template<typename T, concepts::typelist LIST, template<typename> typename PREDICATE, concepts::typelist FRONT = typelist<>>
struct insert_if;

template<typename T, concepts::typelist LIST, template<typename> typename PREDICATE,concepts::typelist FRONT = typelist<>>
using insert_if_t = typename insert_if<T, LIST, PREDICATE, FRONT>::type;

template<typename T, template<typename> typename PREDICATE, concepts::typelist FRONT>
struct insert_if<T, typelist<>, PREDICATE, FRONT>
{
    using type = nil_type;
};

template<typename T, template<typename> typename PREDICATE, concepts::typelist FRONT, typename ELEMENT, typename... RESTs>
struct insert_if<T, typelist<ELEMENT, RESTs...>, PREDICATE, FRONT>
{
    using type = std::conditional_t<PREDICATE<ELEMENT>::value, concat_t<FRONT, typelist<T, ELEMENT, RESTs...>>, insert_if_t<T, typelist<RESTs...>, PREDICATE, append_t<ELEMENT, FRONT>>>;
};


template<typename T, concepts::typelist LIST, template<typename> typename PREDICATE, concepts::typelist FRONT = typelist<>>
struct insert_if_else_append
{
    using inserted = insert_if_t<T, LIST, PREDICATE, FRONT>;
    using type = std::conditional_t<std::is_same_v<inserted, nil_type>, append_t<T, LIST>, inserted>;
};

template<typename T, concepts::typelist LIST, template<typename> typename PREDICATE,concepts::typelist FRONT = typelist<>>
using insert_if_else_append_t = typename insert_if_else_append<T, LIST, PREDICATE, FRONT>::type;

template<concepts::typelist LIST, template<typename, typename> typename COMPARE>
struct sort;

template<concepts::typelist LIST, template<typename, typename> typename COMPARE>
using sort_t = typename sort<LIST, COMPARE>::type;

template<template<typename, typename> typename COMPARE>
struct sort<typelist<>, COMPARE>
{
    using type = typelist<>;
};

template<template<typename, typename> typename COMPARE, typename FIRST, typename... RESTs>
struct sort<typelist<FIRST, RESTs...>, COMPARE>
{
    template<typename T>
    struct comp : COMPARE<FIRST, T> {};
    using type = insert_if_else_append_t<FIRST, sort_t<typelist<RESTs...>, COMPARE>, comp>;
};

} // namespace internal

template<concepts::typelist LIST, template<typename, typename> typename COMPARE>
struct sort
{
    using type = internal::sort_t<LIST, COMPARE>;
};

template<concepts::typelist LIST, template<typename, typename> typename COMPARE>
using sort_t = typename sort<LIST, COMPARE>::type;

template<typename T1, typename T2>
struct gt_size
{
    using type = bool;
    static constexpr type value = sizeof(T1) > sizeof(T2);
};

template<typename T1, typename T2>
inline constexpr bool gt_size_v = gt_size<T1, T2>::value;

template<typename T1, typename T2>
struct lt_size
{
    using type = bool;
    static constexpr type value = sizeof(T1) < sizeof(T2);
};

template<typename T1, typename T2>
inline constexpr bool lt_size_v = lt_size<T1, T2>::value;

} // namespace mtl
