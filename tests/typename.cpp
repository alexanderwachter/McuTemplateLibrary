/*
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/TypeName.hpp>

#include <string_view>

namespace names {

struct plain {};

template<typename T>
struct wrapper {};

namespace nested {
struct inner {};
} // namespace nested

static_assert(mtl::short_name<plain>() == "plain");
static_assert(mtl::short_name<nested::inner>() == "inner");

// template arguments are cut before the namespaces: the last "::" of
// an instantiated name may sit inside an argument
static_assert(mtl::short_name<wrapper<plain>>() == "wrapper");
static_assert(mtl::short_name<wrapper<nested::inner>>() == "wrapper");

// short_name_of is the same name, null-terminated in static storage
static_assert(std::string_view{mtl::short_name_of<plain>} == "plain");
static_assert(std::string_view{mtl::short_name_of<wrapper<nested::inner>>} == "wrapper");
static_assert(mtl::short_name_of<plain>[sizeof("plain") - 1] == '\0');

// values: enumerators keep their enum, class values their type
enum class color { red, green };
struct lamp {
    bool on;
};
struct pair {
    lamp first;
    color second;
};

static_assert(mtl::value_name<color::red>() == "names::color::red");
static_assert(mtl::short_value_name<color::red>() == "color::red");
static_assert(mtl::value_name<lamp{true}>() == "names::lamp{true}");
static_assert(mtl::short_value_name<lamp{true}>() == "lamp{true}");
static_assert(mtl::value_name<nested::inner{}>() == "names::nested::inner()"); // empty: parentheses
static_assert(mtl::short_value_name<nested::inner{}>() == "inner()");
// nested arguments keep the compiler's spelling
static_assert(mtl::short_value_name<pair{{false}, color::green}>() ==
              "pair{names::lamp{false}, names::color::green}");
static_assert(mtl::short_value_name<42>() == "42");
static_assert(mtl::short_value_name<true>() == "true");

// the anonymous-namespace marker is not an initializer
namespace {
enum class hidden { a };
struct box {
    int size;
};
} // namespace
static_assert(mtl::short_value_name<hidden::a>() == "hidden::a");
static_assert(mtl::short_value_name<box{7}>() == "box{7}");

} // namespace names
