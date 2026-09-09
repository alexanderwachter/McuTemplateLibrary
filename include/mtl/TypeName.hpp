/*
 * Compile-time type names, for diagnostics, diagrams, and logging.
 *
 * type_name<T>() is the full name as spelled by the compiler,
 * short_name<T>() strips template arguments and namespace qualifiers
 * for readable labels (mtl::foo<Bar> -> "foo"), and short_name_of<T>
 * is the short name as a null-terminated string in static storage -
 * a persistent char const* for C APIs such as deferred logging.
 *
 * value_name<V>() spells a constant of structural type the way the
 * compiler does (an enumerator as ns::color::red, an aggregate as
 * ns::lamp{true}), short_value_name<V>() without the namespaces
 * (color::red, lamp{true}) - for diagrams showing a state's constexpr
 * annotations.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <array>
#include <cstddef>
#include <string_view>
#include <type_traits>

namespace mtl {

template<typename T>
constexpr std::string_view type_name()
{
#if defined(__GNUC__) || defined(__clang__)
    std::string_view const function = __PRETTY_FUNCTION__;
    auto const start = function.find("T = ") + 4;
    return function.substr(start, function.find_first_of("];", start) - start);
#else
    return "unknown";
#endif
}

// Template arguments are cut before the namespaces: the last "::" of
// an instantiated name may sit inside an argument
template<typename T>
constexpr std::string_view short_name()
{
    auto name = type_name<T>();
    if (auto const angle = name.find('<'); angle != std::string_view::npos) {
        name = name.substr(0, angle);
    }
    if (auto const colon = name.rfind("::"); colon != std::string_view::npos) {
        name = name.substr(colon + 2);
    }
    return name;
}

template<auto VALUE>
constexpr std::string_view value_name()
{
#if defined(__GNUC__) || defined(__clang__)
    std::string_view const function = __PRETTY_FUNCTION__;
    auto const start = function.find("VALUE = ") + 8;
    return function.substr(start, function.find_first_of("];", start) - start);
#else
    return "unknown";
#endif
}

namespace internal {

// Where a class value's initializer starts: the first brace - or
// parenthesis, an empty aggregate is spelled inner() - that is not the
// anonymous-namespace marker ({anonymous} on GCC, (anonymous namespace)
// on Clang)
constexpr std::size_t initializerStart(std::string_view name)
{
    for (auto pos = name.find_first_of("{("); pos != std::string_view::npos;
         pos      = name.find_first_of("{(", pos + 1)) {
        auto const rest = name.substr(pos);
        if (!rest.starts_with("{anonymous}") && !rest.starts_with("(anonymous namespace)")) {
            return pos;
        }
    }
    return std::string_view::npos;
}

} // namespace internal

// An enumerator keeps its enum (color::red), a class value keeps its
// type before the braces (lamp{true}, nested arguments untouched; an
// empty one is spelled inner()), anything else is spelled as is
template<auto VALUE>
constexpr std::string_view short_value_name()
{
    auto name        = value_name<VALUE>();
    auto const brace = internal::initializerStart(name);
    auto const head  = brace == std::string_view::npos ? name : name.substr(0, brace);
    auto colon       = head.rfind("::");
    if (colon == std::string_view::npos) {
        return name;
    }
    if constexpr (std::is_enum_v<decltype(VALUE)>) {
        if (auto const type = head.rfind("::", colon - 1); type != std::string_view::npos) {
            colon = type;
        } else {
            return name;
        }
    }
    return name.substr(colon + 2);
}

namespace internal {

template<typename T>
inline constexpr auto short_name_storage = [] {
    constexpr auto name = short_name<T>();
    std::array<char, name.size() + 1> chars{};
    for (std::size_t i = 0; i < name.size(); ++i) {
        chars[i] = name[i];
    }
    return chars;
}();

} // namespace internal

template<typename T>
inline constexpr char const* short_name_of = internal::short_name_storage<T>.data();

} // namespace mtl
