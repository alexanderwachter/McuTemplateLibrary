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

} // namespace names
