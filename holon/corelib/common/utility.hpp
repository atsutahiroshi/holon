/* utility - Utility header
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of the holon.
 *
 * The holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HOLON_COMMON_UTILITY_HPP_
#define HOLON_COMMON_UTILITY_HPP_

#include <iostream>
#include <type_traits>
#include <utility>

namespace holon {

// compiler detection
#define HOLON_CPP11_OR_GREATER (__cplusplus >= 201103L)
#define HOLON_CPP14_OR_GREATER (__cplusplus >= 201402L)
#define HOLON_CPP17_OR_GREATER (__cplusplus >= 201703L)

template <bool...>
struct bool_pack;

template <bool... Bools>
using all_true =
    std::is_same<bool_pack<true, Bools...>, bool_pack<Bools..., true>>;

template <class Base, class... Derived>
struct all_are_base_of {
#if HOLON_CPP17_OR_GREATER
  static constexpr bool value = (... && std::is_base_of<Base, Derived>::value);
#else
  static constexpr bool value =
      all_true<std::is_base_of<Base, Derived>::value...>::value;
#endif
};

#if HOLON_CPP17_OR_GREATER
template <typename T>
using as_const<T> = std::as_const<T>;
#else
template <typename T>
constexpr typename std::add_const<T>::type& as_const(T& t) noexcept {
  return t;
}
#endif

template <std::size_t... Elem>
struct index_seq {
  static constexpr std::size_t num = sizeof...(Elem);
  template <std::size_t I>
  static constexpr std::size_t get() {
    static_assert(I < num, "index_seq: out of range.");
    return elem[I];
  }

 private:
  static constexpr std::size_t elem[] = {Elem...};
};

template <std::size_t N, typename T>
struct make_index_seq_impl;

template <std::size_t N, std::size_t... I>
struct make_index_seq_impl<N, index_seq<I...>> {
  using type =
      typename make_index_seq_impl<N - 1, index_seq<N - 1, I...>>::type;
};

template <std::size_t... I>
struct make_index_seq_impl<0, index_seq<I...>> {
  using type = index_seq<I...>;
};

template <std::size_t N>
using make_index_seq = typename make_index_seq_impl<N, index_seq<>>::type;

template <class Ch, class Tr, class Tuple, std::size_t... Is>
void print_tuple(std::basic_ostream<Ch, Tr>& os, const Tuple& t,
                 index_seq<Is...>) {
  using dummy = int[];
  (void)dummy{0, (void(os << (Is == 0 ? "" : ", ") << std::get<Is>(t)), 0)...};
}

template <class...>
struct void_helper {
  using type = void;
};
template <class... Args>
using void_t = typename void_helper<Args...>::type;

#define HOLON_GENERATE_MEMBER_TYPE_CHECKER(type) \
  template <class T, class = void>               \
  struct has_##type : std::false_type {};        \
  template <class T>                             \
  struct has_##type<T, void_t<typename T::type>> : std::true_type {}

#define HOLON_HAS_MEMBER_TYPE(class, type) has_##type<class>::value

#if 0
#define HOLON_GENERATE_MEMBER_VAR_CHECKER(var)                 \
  template <class T, class = void>                             \
  struct has_##var : std::false_type {};                       \
  template <class T>                                           \
  struct has_##var<T, decltype(std::declval<T>().var, void())> \
      : std::true_type {}
#else
#define HOLON_GENERATE_MEMBER_VAR_CHECKER(var) \
  template <class T, typename = int>           \
  struct has_##var : std::false_type {};       \
  template <class T>                           \
  struct has_##var<T, decltype((void)T::var, 0)> : std::true_type {}
#endif

#define HOLON_HAS_MEMBER_VAR(class, var) has_##var<class>::value

}  // namespace holon

#endif  // HOLON_COMMON_UTILITY_HPP_
