/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <type_traits>
#include <cstdint>
#include <ratio>

namespace mtl {

template<typename>
struct is_unit : std::false_type {};

template<typename T>
inline constexpr bool is_unit_v = is_unit<T>::value;

template<typename>
struct is_quantity : std::false_type {};

template<typename T>
inline constexpr bool is_quantity_v = is_quantity<T>::value;

template<typename>
struct is_ratio : std::false_type {};

template<std::intmax_t Num, std::intmax_t Denom >
struct is_ratio<std::ratio<Num, Denom>> : std::true_type {};

template<typename T>
inline constexpr bool is_ratio_v = is_ratio<T>::value;

namespace concepts
{
template<typename T>
concept unit = is_unit_v<T>;

template<typename T>
concept ratio = is_ratio_v<T>;

template<typename T>
concept quantity = is_quantity_v<T>;
} // namespace concepts

template<typename>
struct get_unit;

template<typename T>
using get_unit_t = typename get_unit<T>::type;

template<typename>
struct get_scaling;

template<typename T>
using get_scaling_t = typename get_scaling<T>::type;

template<concepts::unit UNIT, concept::ratio SCALING>
struct quantity {};

namespace concepts
{
template<typename T>
concept quantity = is_quantity_v<T>;
} // namespace concepts

template<concepts::unit UNIT, concept::ratio SCALING>
struct get_unit<quantity<UNIT, SCALING>> : std::type_identity<UNIT> {};

template<concepts::unit UNIT, concept::ratio SCALING>
struct get_scaling<quantity<UNIT, SCALING>> : std::type_identity<SCALING> {};

template<concepts::quantity QUANTITY, std::integral VALUE_TYPE>
class Value
{
public:
    constexpr explicit Value(VALUE_TYPE v) noexcept : value(v) {}

    constexpr VALUE_TYPE count() const noexcept { return value; }

    template<concepts::quantity TARGET>
    Value<TARGET, VALUE_TYPE> to() const noexcept
    {
        using scaling = std::ratio_divide_t<get_scaling_t<QUANTITY>, get_scaling_t<TARGET>>;
        return Value<TARGET, VALUE_TYPE>(value * scaling::num / scaling::den);
    }

private:
    VALUE_TYPE value;
};


struct voltage {};
struct current {};

struct is_unit<voltage> : std::true_type {};
struct is_unit<current> : std::true_type {};

} // namespace mtl
