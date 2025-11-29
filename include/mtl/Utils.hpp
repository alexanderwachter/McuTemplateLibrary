/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <bit>

namespace mtl {

template <unsigned int>
struct width_to_uint_exact;

template<unsigned int WIDTH>
using width_to_uint_exact_t = typename width_to_uint_exact<WIDTH>::type;

template <> struct width_to_uint_exact<1> { using type = bool; };
template <> struct width_to_uint_exact<8> { using type = std::uint8_t; };
template <> struct width_to_uint_exact<16> { using type = std::uint16_t; };
template <> struct width_to_uint_exact<32> { using type = std::uint32_t; };
template <> struct width_to_uint_exact<64> { using type = std::uint64_t; };


template <unsigned int WIDTH>
struct width_to_uint
{
    using type = width_to_uint_exact_t<std::bit_ceil((WIDTH + 7U) / 8U) * 8U>;
};

template<>
struct width_to_uint<1>
{
    using type = width_to_uint_exact_t<1>;
};

template <unsigned int WIDTH>
using width_to_uint_t = typename width_to_uint<WIDTH>::type;

template <unsigned int>
struct width_to_int_exact;

template<unsigned int WIDTH>
using width_to_int_exact_t = typename width_to_int_exact<WIDTH>::type;

template <> struct width_to_int_exact<1> { using type = bool; };
template <> struct width_to_int_exact<8> { using type = std::int8_t; };
template <> struct width_to_int_exact<16> { using type = std::int16_t; };
template <> struct width_to_int_exact<32> { using type = std::int32_t; };
template <> struct width_to_int_exact<64> { using type = std::int64_t; };

template <unsigned int WIDTH>
struct width_to_int
{
    using type = width_to_int_exact_t<std::bit_ceil((WIDTH + 7U) / 8U) * 8U>;
};

template <unsigned int WIDTH>
using width_to_int_t = typename width_to_int<WIDTH>::type;

} // namespace mtl
