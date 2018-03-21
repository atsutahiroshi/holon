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

}  // namespace holon

#endif  // HOLON_COMMON_UTILITY_HPP_
