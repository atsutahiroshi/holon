/* misc - miscellanies related to mathematics.
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

#ifndef HOLON_MATH_MISC_HPP_
#define HOLON_MATH_MISC_HPP_

#include <algorithm>
#include <limits>
#include <type_traits>

namespace holon {

template <typename T>
int sgn(const T& x) {
  return (T(0) < x) - (x < T(0));
}

template <typename T>
T square(const T& x) {
  return x * x;
}

template <typename T>
T limit(const T& x, const T& lower, const T& upper) {
  return x <= lower ? lower : ((x >= upper) ? upper : x);
}

template <typename T>
T bound(const T& x, T b1, T b2) {
  if (b1 > b2) std::swap(b1, b2);
  return limit<T>(x, b1, b2);
}

template <typename T>
struct is_float_type
    : std::integral_constant<bool, std::is_same<T, float>::value ||
                                       std::is_same<T, double>::value ||
                                       std::is_same<T, long double>::value> {};

template <typename T>
bool is_tiny(const T& x) {
  static_assert(is_float_type<T>::value,
                "is_tiny must be used with floating point type");
  return std::abs(x) < std::numeric_limits<T>::epsilon();
}

template <typename T>
bool is_positive(const T& x) {
  return x > T(0);
}

template <typename T>
bool is_negative(const T& x) {
  return x < T(0);
}

}  // namespace holon

#endif  // HOLON_MATH_MISC_HPP_
