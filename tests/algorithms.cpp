/*
 * Copyright (c) 2025 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mtl/TypelistAlgorithms.hpp>
#include <type_traits>
#include <cstdint>

using namespace mtl;

struct test_type {};

struct test1 : test_type {};
struct test2 : test_type {};
struct test3 : test_type {};
struct test4 : test_type {};

using empty = typelist<>;
using all = typelist<test1, test2, test3, test4>;
using first = typelist<test1>;
using second = typelist<test2>;
using firstTwo = typelist<test1, test2>;
using secondTwo = typelist<test3, test4>;
using firstFour = typelist<test1, test2, test3, test4>;

template<typename T>
struct is_test1 : std::is_same<T, test1> {};

namespace AllOf {
    static_assert(all_of_v<typelist<bool, char, int>, std::is_integral> == true);
    static_assert(all_of_v<typelist<float, char, int>, std::is_integral> == false);
    static_assert(all_of_v<typelist<bool, float, int>, std::is_integral> == false);
    static_assert(all_of_v<typelist<float, char, int>, std::is_integral> == false);
    static_assert(all_of_v<empty, std::is_integral> == true);
}

namespace AnyOf {
    static_assert(any_of_v<typelist<float, void, double>, std::is_integral> == false);
    static_assert(any_of_v<typelist<int, void, float>, std::is_integral> == true);
    static_assert(any_of_v<typelist<void, int, float>, std::is_integral> == true);
    static_assert(any_of_v<typelist<void, float, int>, std::is_integral> == true);
    static_assert(any_of_v<empty, std::is_integral> == false);
}

namespace NoneOf {
    static_assert(none_of_v<typelist<float, void, double>, std::is_integral> == true);
    static_assert(none_of_v<typelist<int, void, float>, std::is_integral> == false);
    static_assert(none_of_v<typelist<void, int, float>, std::is_integral> == false);
    static_assert(none_of_v<typelist<void, float, int>, std::is_integral> == false);
    static_assert(none_of_v<empty, std::is_integral> == true);
}

namespace HasA {
    static_assert(has_a_v<typelist<float, void, double>,char> == false);
    static_assert(has_a_v<typelist<int, void, float>, int> == true);
    static_assert(has_a_v<typelist<int, void, float>, void> == true);
    static_assert(has_a_v<typelist<int, void, float>, float> == true);
    static_assert(has_a_v<empty, void> == false);
}

namespace Sum {
    using val1 = std::integral_constant<char, 1>;
    using val2 = std::integral_constant<int, 2>;
    using val3 = std::integral_constant<int, 3>;

    static_assert(sum_v<empty> == 0);
    static_assert(sum_v<typelist<val1, val2, val3>> == 6);
    static_assert(std::is_same_v<typename sum<empty>::value_type, int>);
}

namespace FindIf {
    static_assert(std::is_same_v<find_if_t<typelist<float, void, double>, std::is_integral>, nil_type>);
    static_assert(std::is_same_v<find_if_t<typelist<float, int, double>, std::is_integral>, int>);
    static_assert(std::is_same_v<find_if_t<typelist<float, void, int>, std::is_integral>, int>);
    static_assert(std::is_same_v<find_if_t<typelist<int, void, float>, std::is_integral>, int>);
    static_assert(std::is_same_v<find_if_t<typelist<void, char, int>, std::is_integral>, char>);
}

namespace MinMax {
    static_assert(min_t<typelist<std::integral_constant<int, -1>>>::value == -1);
    static_assert(min_t<typelist<std::integral_constant<int, 0>, std::integral_constant<int, 1>, std::integral_constant<int, 2>>>::value == 0);
    static_assert(min_t<typelist<std::integral_constant<int, 0>, std::integral_constant<int, 1>, std::integral_constant<int, -1>>>::value == -1);
    static_assert(min_t<typelist<std::integral_constant<int, 0>, std::integral_constant<int, -1>, std::integral_constant<int, 2>>>::value == -1);

    static_assert(max_t<typelist<std::integral_constant<int, 1>>>::value == 1);
    static_assert(max_t<typelist<std::integral_constant<int, 0>, std::integral_constant<int, 1>, std::integral_constant<int, 2>>>::value == 2);
    static_assert(max_t<typelist<std::integral_constant<int, 3>, std::integral_constant<int, 1>, std::integral_constant<int, 2>>>::value == 3);
    static_assert(max_t<typelist<std::integral_constant<int, 0>, std::integral_constant<int, -1>, std::integral_constant<int, 2>>>::value == 2);
}

namespace Filter {
    static_assert(std::is_same_v<filter_t<typelist<>, std::is_integral>, empty>);
    static_assert(std::is_same_v<filter_t<typelist<float, void, double>, std::is_integral>, empty>);
    static_assert(std::is_same_v<filter_t<typelist<bool, void, char>, std::is_integral>, typelist<bool, char>>);
    static_assert(std::is_same_v<filter_t<typelist<bool, void, char, void>, std::is_integral>, typelist<bool, char>>);
}

namespace RemoveIf {
    static_assert(std::is_same_v<remove_if_t<typelist<>, std::is_integral>, empty>);
    static_assert(std::is_same_v<remove_if_t<typelist<float, void, double>, std::is_integral>, typelist<float, void, double>>);
    static_assert(std::is_same_v<remove_if_t<typelist<bool, void, char>, std::is_integral>, typelist<void>>);
    static_assert(std::is_same_v<remove_if_t<typelist<bool, void, char, void>, std::is_integral>, typelist<void, void>>);
}

namespace CountIf {
    static_assert(count_if_v<typelist<>, std::is_integral> == 0U);
    static_assert(count_if_v<typelist<float, void, double>, std::is_integral> == 0U);
    static_assert(count_if_v<typelist<bool, void, char>, std::is_integral> == 2U);
    static_assert(count_if_v<typelist<void, int, void>, std::is_integral> == 1U);
}


namespace Transform {
    static_assert(std::is_same_v<transform_t<typelist<>, std::make_unsigned>, typelist<>>);
    static_assert(std::is_same_v<transform_t<typelist<char, int, long>, std::make_unsigned>, typelist<unsigned char, unsigned int, unsigned long>>);
}

namespace Unique {
    static_assert(std::is_same_v<unique_t<typelist<>>, typelist<>>);
    static_assert(std::is_same_v<unique_t<typelist<bool, bool>>, typelist<bool>>);
    static_assert(std::is_same_v<unique_t<typelist<test1, test2, test1, test3, test2, test4>>, typelist<test1, test2, test3, test4>>);
}

namespace InsertIf {
    template<typename T>
    struct is_test2 : std::is_same<T, test2> {};

    static_assert(std::is_same_v<internal::insert_if_t<char, empty, is_test2>, nil_type>);
    static_assert(std::is_same_v<internal::insert_if_else_append_t<char, empty, is_test2>, typelist<char>>);
    static_assert(std::is_same_v<internal::insert_if_t<test4, secondTwo, is_test2>, nil_type>);
    static_assert(std::is_same_v<internal::insert_if_t<test4, firstTwo, is_test2>, typelist<test1, test4, test2>>);
    static_assert(std::is_same_v<internal::insert_if_t<test4, typelist<test1, test2, test3>, is_test2>, typelist<test1, test4, test2, test3>>);
} // namespace InsertIf

namespace Sort {
    static_assert(gt_size_v<int, char> == true);
    static_assert(gt_size_v<char, int> == false);
    static_assert(lt_size_v<int, char> == false);
    static_assert(lt_size_v<char, int> == true);
    static_assert(std::is_same_v<sort_t<empty, gt_size>, empty>);
    static_assert(std::is_same_v<sort_t<typelist<char>, gt_size>, typelist<char>>);

    using list_unsorted = typelist<std::uint8_t, std::uint32_t, std::uint64_t, std::uint16_t>;
    using list_asc = typelist<std::uint8_t, std::uint16_t, std::uint32_t, std::uint64_t>;
    using list_desc = typelist<std::uint64_t, std::uint32_t, std::uint16_t, std::uint8_t>;
    static_assert(std::is_same_v<sort_t<list_unsorted, gt_size>, list_desc>);
    static_assert(std::is_same_v<sort_t<list_asc, gt_size>, list_desc>);
    static_assert(std::is_same_v<sort_t<list_desc, gt_size>, list_desc>);
    static_assert(std::is_same_v<sort_t<list_unsorted, lt_size>, list_asc>);
} // namespace Sort
