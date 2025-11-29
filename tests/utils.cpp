/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/Utils.hpp>
#include <type_traits>

using namespace mtl;

namespace width_to_uint {

static_assert(std::is_same_v<width_to_uint_t<1>, bool>);
static_assert(std::is_same_v<width_to_uint_t<2>, std::uint8_t>);
static_assert(std::is_same_v<width_to_uint_t<7>, std::uint8_t>);
static_assert(std::is_same_v<width_to_uint_t<8>, std::uint8_t>);
static_assert(std::is_same_v<width_to_uint_t<9>, std::uint16_t>);
static_assert(std::is_same_v<width_to_uint_t<16>, std::uint16_t>);
static_assert(std::is_same_v<width_to_uint_t<17>, std::uint32_t>);
static_assert(std::is_same_v<width_to_uint_t<32>, std::uint32_t>);
static_assert(std::is_same_v<width_to_uint_t<33>, std::uint64_t>);
static_assert(std::is_same_v<width_to_uint_t<64>, std::uint64_t>);

static_assert(std::is_same_v<width_to_int_t<2>, std::int8_t>);
static_assert(std::is_same_v<width_to_int_t<7>, std::int8_t>);
static_assert(std::is_same_v<width_to_int_t<8>, std::int8_t>);
static_assert(std::is_same_v<width_to_int_t<9>, std::int16_t>);
static_assert(std::is_same_v<width_to_int_t<16>, std::int16_t>);
static_assert(std::is_same_v<width_to_int_t<17>, std::int32_t>);
static_assert(std::is_same_v<width_to_int_t<32>, std::int32_t>);
static_assert(std::is_same_v<width_to_int_t<33>, std::int64_t>);
static_assert(std::is_same_v<width_to_int_t<64>, std::int64_t>);

} // namespace width_to_uint