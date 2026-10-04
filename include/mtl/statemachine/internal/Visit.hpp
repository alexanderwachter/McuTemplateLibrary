/*
 * fsm: calling a visitor with the active alternative of a variant
 *
 * Copyright (c) 2026 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstddef>
#include <type_traits>
#include <utility>
#include <variant>

namespace fsm::internal {

// A fold in place of std::visit: every visitor instantiation is
// inlinable and no function-pointer table or bad_variant_access path
// can be emitted. A valueless variant matches no alternative and
// yields a value-initialized result (unreachable in process(): the
// states can never make the variant valueless).
// std::visit measured 3.7 kB larger on arm-zephyr-eabi GCC 14.3 -Os
// (Cortex-M0+, 14-state/20-event machine) - it emits per-(event,
// state) invoke thunks and tables that dominate at scale - and 32
// bytes .text smaller on GCC 15.2 x86-64 -Os (traffic_light.cpp).
// The visitor answers every alternative with the type it answers the
// first with
template<typename VISITOR, typename FIRST, typename... ALTERNATIVEs>
constexpr auto visit(VISITOR&& visitor, std::variant<FIRST, ALTERNATIVEs...>& variant)
{
    using result_type = std::remove_cvref_t<decltype(visitor(std::declval<FIRST&>()))>;
    return [&]<std::size_t... INDEXs>(std::index_sequence<INDEXs...>) {
        result_type result{};
        static_cast<void>(((variant.index() == INDEXs &&
                            (result = visitor(*std::get_if<INDEXs>(&variant)), true)) ||
                           ...));
        return result;
    }(std::index_sequence_for<FIRST, ALTERNATIVEs...>{});
}

} // namespace fsm::internal
