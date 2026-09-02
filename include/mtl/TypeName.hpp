/*
 * Compile-time type names, for diagnostics, diagrams, and logging.
 *
 * type_name<T>() is the full name as spelled by the compiler,
 * short_name<T>() strips template arguments and namespace qualifiers
 * for readable labels (mtl::foo<Bar> -> "foo"), and short_name_of<T>
 * is the short name as a null-terminated string in static storage -
 * a persistent char const* for C APIs such as deferred logging.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Alexander Wachter
 */

#pragma once

#include <array>
#include <cstddef>
#include <string_view>

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
