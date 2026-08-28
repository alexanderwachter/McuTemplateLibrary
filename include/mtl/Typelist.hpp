/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <type_traits>
#include <cstddef>
#include <utility>

// Conventions:
// template arguments are written in all capital letters
// template packs end with a lowercase s
// A result of a trait is a type alias in the class called type, or a static constexpr value member called value (or both) (same as std).
// For type traits containing an alias called type, a type alias exist ending with <trait name>_t that expose the ::type (same as std)
// For type traits containing a value member, a constexpr value exist ending with <trait name>_v with the same value (same as std)

namespace mtl {

// Trait that counts elements
template<typename T>
struct count;

template<typename T>
inline constexpr std::size_t count_v = count<T>::value;

// A type that holds any number of types
template<typename... T>
struct typelist {};

// A trait that is the true_type if the template parameter is a typelist, else false_type
template<typename T>
struct is_typelist : std::false_type {};

template<typename... ELEMENTs>
struct is_typelist<typelist<ELEMENTs...>> : std::true_type {};

template<typename T>
inline constexpr bool is_typelist_v = is_typelist<T>::value;

namespace concepts
{
template<typename T>
concept typelist = is_typelist_v<T>;
} // namespace concepts

// A type that is the result if an algorithm has no result
struct nil_type {};

namespace internal {

// Always false, but dependent on the template arguments so that a static_assert
// using it only fires when the enclosing template is actually instantiated
template<typename...>
inline constexpr bool always_false = false;

} // namespace internal

// Specialization of count for the typelist. Counts the number of elements in the typelist
template<typename... T>
struct count<typelist<T...>> : std::integral_constant<std::size_t, sizeof...(T)> {};

// Trait to append an element to the end of the typelist
template<typename T, concepts::typelist LIST>
struct append;

template<typename T, concepts::typelist LIST>
using append_t = typename append<T, LIST>::type;

template<typename T, typename... ELEMENTs>
struct append<T, typelist<ELEMENTs...>>
{
    using type = typelist<ELEMENTs..., T>;
};

// Trait to prepend an element at the beginning of the typelist
template<typename T, concepts::typelist LIST>
struct prepend;

template<typename T, concepts::typelist LIST>
using prepend_t = typename prepend<T, LIST>::type;

template<typename T, typename... ELEMENTs>
struct prepend<T, typelist<ELEMENTs...>>
{
    using type = typelist<T, ELEMENTs...>;
};

// Trait to concatenate two typelists
template<concepts::typelist LIST1, concepts::typelist LIST2>
struct concat;

template<concepts::typelist LIST1, concepts::typelist LIST2>
using concat_t = typename concat<LIST1, LIST2>::type;

template<typename... ELEMENT1s, typename... ELEMENT2s>
struct concat<typelist<ELEMENT1s...>, typelist<ELEMENT2s...>>
{
    using type = typelist<ELEMENT1s..., ELEMENT2s...>;
};

// Trait to get a list in reversed order
template<concepts::typelist LIST>
struct reverse;

template<concepts::typelist LIST>
using reverse_t = typename reverse<LIST>::type;

template<>
struct reverse<typelist<>>: std::type_identity<typelist<>> {};

template<typename FIRST, typename... RESTs>
struct reverse<typelist<FIRST, RESTs...>>
{
    using type = append_t<FIRST, reverse_t<typelist<RESTs...>>>;
};

// Trait to remove the first element of the typelist. An empty list stays empty
template<concepts::typelist LIST>
struct remove_front;

template<concepts::typelist LIST>
using remove_front_t = typename remove_front<LIST>::type;

template<>
struct remove_front<typelist<>> : std::type_identity<typelist<>> {};

template<typename FIRST, typename... RESTs>
struct remove_front<typelist<FIRST, RESTs...>>
{
    using type = typelist<RESTs...>;
};

// Trait to remove the last element of the typelist. An empty list stays empty
template<concepts::typelist LIST>
struct remove_back;

template<concepts::typelist LIST>
using remove_back_t = typename remove_back<LIST>::type;

template<>
struct remove_back<typelist<>> : std::type_identity<typelist<>> {};

template<typename LAST>
struct remove_back<typelist<LAST>> : std::type_identity<typelist<>> {};

template<typename FIRST, typename... RESTs>
struct remove_back<typelist<FIRST, RESTs...>>
{
    using type = prepend_t<FIRST, remove_back_t<typelist<RESTs...>>>;
};

namespace internal {

// PROCESSED holds the elements before CURRENT_INDEX, REMAINING the ones from CURRENT_INDEX on
template<std::size_t INDEX, std::size_t CURRENT_INDEX, concepts::typelist PROCESSED, concepts::typelist REMAINING>
struct remove_helper;

template<std::size_t INDEX, std::size_t CURRENT_INDEX, concepts::typelist PROCESSED, typename FIRST, typename... RESTs>
struct remove_helper<INDEX, CURRENT_INDEX, PROCESSED, typelist<FIRST, RESTs...>>
{
    using type = typename remove_helper<INDEX, CURRENT_INDEX + 1, append_t<FIRST, PROCESSED>, typelist<RESTs...>>::type;
};

template<std::size_t INDEX, concepts::typelist PROCESSED, typename FIRST, typename... RESTs>
struct remove_helper<INDEX, INDEX, PROCESSED, typelist<FIRST, RESTs...>>
{
    using type = concat_t<PROCESSED, typelist<RESTs...>>;
};

} // namespace internal

// Trait to remove the element at the given index of the typelist
template<std::size_t INDEX, concepts::typelist LIST>
struct remove_at
{
    static_assert(INDEX < count_v<LIST>, "index out of range");
    using type = typename internal::remove_helper<INDEX, 0, typelist<>, LIST>::type;
};

template<std::size_t INDEX, concepts::typelist LIST>
using remove_at_t = typename remove_at<INDEX, LIST>::type;


// Get the first type from the list
template<concepts::typelist LIST>
struct front;

template<concepts::typelist LIST>
using front_t = typename front<LIST>::type;

template<concepts::typelist LIST>
    requires (count_v<LIST> == 0U)
struct front<LIST>
{
    static_assert(internal::always_false<LIST>, "front of an empty typelist");
};

template<typename FIRST, typename... RESTs>
struct front<typelist<FIRST, RESTs...>>
{
    using type = FIRST;
};

// Get the last type from the list
template<concepts::typelist LIST>
struct back;

template<concepts::typelist LIST>
using back_t = typename back<LIST>::type;

template<concepts::typelist LIST>
    requires (count_v<LIST> == 0U)
struct back<LIST>
{
    static_assert(internal::always_false<LIST>, "back of an empty typelist");
};

template<typename LAST>
struct back<typelist<LAST>>
{
    using type = LAST;
};

template<typename FIRST, typename... RESTs>
struct back<typelist<FIRST, RESTs...>>
{
    using type = back_t<typelist<RESTs...>>;
};


#if defined(__has_builtin)
#  if __has_builtin(__type_pack_element)
#    define MTL_HAS_TYPE_PACK_ELEMENT 1
#  endif
#endif
#ifndef MTL_HAS_TYPE_PACK_ELEMENT
#  define MTL_HAS_TYPE_PACK_ELEMENT 0
#endif

#if !MTL_HAS_TYPE_PACK_ELEMENT
namespace internal {

template<std::size_t INDEX, concepts::typelist LIST>
struct at_helper;

template<typename FIRST, typename... RESTs>
struct at_helper<0U, typelist<FIRST, RESTs...>>
{
    using type = FIRST;
};

template<std::size_t INDEX, typename FIRST, typename... RESTs>
struct at_helper<INDEX, typelist<FIRST, RESTs...>> : at_helper<INDEX - 1U, typelist<RESTs...>> {};

// Skips eight elements per instantiation to keep the recursion depth low
template<std::size_t INDEX, typename ELEMENT0, typename ELEMENT1, typename ELEMENT2, typename ELEMENT3,
         typename ELEMENT4, typename ELEMENT5, typename ELEMENT6, typename ELEMENT7, typename... RESTs>
    requires (INDEX >= 8U)
struct at_helper<INDEX, typelist<ELEMENT0, ELEMENT1, ELEMENT2, ELEMENT3, ELEMENT4, ELEMENT5, ELEMENT6, ELEMENT7, RESTs...>>
    : at_helper<INDEX - 8U, typelist<RESTs...>> {};

} // namespace internal
#endif


// Get the type at the given index of the list
template<std::size_t INDEX, concepts::typelist LIST>
struct at;

template<std::size_t INDEX, typename... ELEMENTs>
struct at<INDEX, typelist<ELEMENTs...>>
{
    static_assert(INDEX < sizeof...(ELEMENTs), "index out of range");
#if MTL_HAS_TYPE_PACK_ELEMENT
    using type = __type_pack_element<INDEX, ELEMENTs...>;
#else
    using type = typename internal::at_helper<INDEX, typelist<ELEMENTs...>>::type;
#endif
};

template<std::size_t INDEX, concepts::typelist LIST>
using at_t = typename at<INDEX, LIST>::type;

// get the index of the element in the list
template<typename ELEMENT, concepts::typelist LIST>
struct index_of;

template<typename ELEMENT, concepts::typelist LIST>
inline constexpr std::size_t index_of_v = index_of<ELEMENT, LIST>::value;

template<typename ELEMENT>
struct index_of<ELEMENT, typelist<>>
{
    static_assert(internal::always_false<ELEMENT>, "type not found in list");
};

template<typename ELEMENT, typename... RESTs>
struct index_of<ELEMENT, typelist<ELEMENT, RESTs...>> : std::integral_constant<std::size_t, 0U> {};

template<typename ELEMENT, typename FIRST, typename... RESTs>
struct index_of<ELEMENT, typelist<FIRST, RESTs...>> : std::integral_constant<std::size_t, index_of_v<ELEMENT, typelist<RESTs...>> + 1U> {};


// The common type of the nested ::type of all elements
template<concepts::typelist LIST>
struct common_type;

template<concepts::typelist LIST>
using common_type_t = typename common_type<LIST>::type;

template<typename... ELEMENTs>
struct common_type<typelist<ELEMENTs...>>
{
    using type = std::common_type_t<typename ELEMENTs::type...>;
};

// The common type of the ::value of all elements
template<concepts::typelist LIST>
struct common_value_type;

template<concepts::typelist LIST>
using common_value_type_t = typename common_value_type<LIST>::type;

template<typename... ELEMENTs>
struct common_value_type<typelist<ELEMENTs...>>
{
    using type = std::common_type_t<decltype(ELEMENTs::value)...>;
};

// Flatten a typelist of nested typelists into a single typelist
template<concepts::typelist LIST>
struct linearize;

template<concepts::typelist LIST>
using linearize_t = typename linearize<LIST>::type;

template<>
struct linearize<typelist<>> : std::type_identity<typelist<>> {};

template<typename FIRST, typename... RESTs>
struct linearize<typelist<FIRST, RESTs...>>
{
    using type = prepend_t<FIRST, linearize_t<typelist<RESTs...>>>;
};

template<concepts::typelist FIRST, typename... RESTs>
struct linearize<typelist<FIRST, RESTs...>>
{
    using type = concat_t<linearize_t<FIRST>, linearize_t<typelist<RESTs...>>>;
};

// Rebind the elements of the typelist into another template
template<concepts::typelist LIST, template<typename...> typename TARGET>
struct rebind;

template<concepts::typelist LIST, template<typename...> typename TARGET>
using rebind_t = typename rebind<LIST, TARGET>::type;

template<typename... ELEMENTs, template<typename...> typename TARGET>
struct rebind<typelist<ELEMENTs...>, TARGET>
{
    using type = TARGET<ELEMENTs...>;
};

} // namespace mtl
