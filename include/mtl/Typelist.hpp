/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <type_traits>
#include <cstddef>
#include <concepts>

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
};

// A type that is the result if an algorithm has no result
struct nil_type {};

// Specialization of count for the typelist. Counts the number of elements in the typelist
template<typename... T>
struct count<typelist<T...>> : std::integral_constant<std::size_t, sizeof...(T)> {};

// Trait to append an element to the end of the typelist
template<typename T, concepts::typelist LIST>
struct append;

template<typename T, concepts::typelist LIST>
using append_t = typename append<T, LIST>::type;

template<typename T, typename... ELEMENTs>
struct append<T, typelist<ELEMENTs...>> {
    using type = typelist<ELEMENTs..., T>;
};

// Trait to prepend an element at the beginning of the typelist
template<typename T, concepts::typelist LIST>
struct prepend;

template<typename T, concepts::typelist LIST>
using prepend_t = typename prepend<T, LIST>::type;

template<typename T, typename... ELEMENTs>
struct prepend<T, typelist<ELEMENTs...>> {
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

template<concepts::typelist LIST, typename ENABLE = void>
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

template<std::size_t INDEX, std::size_t CURRENT_INDEX, concepts::typelist LIST, typename FIRST, typename ...REST>
struct remove_helper;

template<std::size_t INDEX, std::size_t CURRENT_INDEX, concepts::typelist LIST, typename FIRST, typename ...REST>
struct remove_helper<INDEX, CURRENT_INDEX, LIST, typelist<FIRST, REST...>>
{
    using type = typename remove_helper<INDEX, CURRENT_INDEX + 1, append_t<FIRST, LIST>, typelist<REST...>>::type;
};

template<std::size_t INDEX, concepts::typelist LIST, typename FIRST, typename ...REST>
struct remove_helper<INDEX, INDEX, LIST, typelist<FIRST, REST...>>
{
    using type = concat_t<LIST, typelist<REST...>>;
};

} // namespace internal

template<std::size_t INDEX, concepts::typelist LIST>
struct remove_at
{
    static_assert(INDEX < count_v<LIST>);
    using type = typename internal::remove_helper<INDEX, 0, typelist<>, LIST>::type;
};

template<std::size_t INDEX, concepts::typelist LIST>
using remove_at_t = typename remove_at<INDEX, LIST>::type;


//Get the first type from the list
template<concepts::typelist LIST>
struct front;

template<concepts::typelist LIST>
using front_t = typename front<LIST>::type;

template<typename FIRST, typename... RESTs>
struct front<typelist<FIRST, RESTs...>>
{
    using type = FIRST;
};

//Get the first type from the list
template<concepts::typelist LIST>
struct back;

template<concepts::typelist LIST>
using back_t = typename back<LIST>::type;

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


namespace internal {

template<std::size_t INDEX, std::size_t CURRENT_INDEX, concepts::typelist LIST>
struct at_helper;

template<std::size_t INDEX, std::size_t CURRENT_INDEX, typename FIRST, typename... REST>
struct at_helper<INDEX, CURRENT_INDEX, typelist<FIRST, REST...>>
{
    using type = typename at_helper<INDEX, CURRENT_INDEX + 1, typelist<REST...>>::type;
};

template<std::size_t INDEX, typename FIRST, typename... REST>
struct at_helper<INDEX, INDEX, typelist<FIRST, REST...>>
{
    using type = FIRST;
};

} // namespace internal


template<std::size_t INDEX, concepts::typelist LIST>
struct at
{
    static_assert(INDEX < count_v<LIST>);
    using type = typename internal::at_helper<INDEX, 0, LIST>::type;
};

template<std::size_t INDEX, concepts::typelist LIST>
using at_t = typename at<INDEX, LIST>::type;

// get the index of the element in the list
template<typename ELEMENT, concepts::typelist LIST>
struct index_of;

template<typename ELEMENT, concepts::typelist LIST>
inline constexpr std::size_t index_of_v = index_of<ELEMENT, LIST>::value;

template<typename ELEMENT>
struct index_of<ELEMENT, typelist<>> {
    static_assert(false, "type not found in list");
};

template<typename ELEMENT, typename... RESTs>
struct index_of<ELEMENT, typelist<ELEMENT, RESTs...>> : std::integral_constant<std::size_t, 0U> {};

template<typename ELEMENT, typename FIRST, typename... RESTs>
struct index_of<ELEMENT, typelist<FIRST, RESTs...>> : std::integral_constant<std::size_t, index_of_v<ELEMENT, typelist<RESTs...>> + 1U> {};


template<concepts::typelist LIST>
struct common_type;

template<concepts::typelist LIST>
using common_type_t = typename common_type<LIST>::type;

template<typename... ELEMENTs>
struct common_type<typelist<ELEMENTs...>>
{
    using type = std::common_type_t<typename ELEMENTs::type...>;
};

template<concepts::typelist LIST>
struct common_value_type;

template<concepts::typelist LIST>
using common_value_type_t = typename common_value_type<LIST>::type;

template<typename... ELEMENTs>
struct common_value_type<typelist<ELEMENTs...>>
{
    using type = std::common_type_t<decltype(ELEMENTs::value)...>;
};

template<typename>
struct linearize;

template<typename T>
using linearize_t = typename linearize<T>::type;

template<>
struct linearize<typelist<>>
{
    using type = typelist<>;
};

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

} // namespace mtl
