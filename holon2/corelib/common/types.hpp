/* types - type definitions
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

#ifndef HOLON_COMMON_TYPES_HPP_
#define HOLON_COMMON_TYPES_HPP_

#include <array>
#include <complex>
#include "holon2/corelib/common/optional.hpp"

namespace holon {

// type definitions of array
template <typename T>
using Array2 = std::array<T, 2>;
template <typename T>
using Array3 = std::array<T, 3>;
template <typename T>
using Array4 = std::array<T, 4>;
template <typename T>
using Array6 = std::array<T, 6>;

using Array2d = Array2<double>;
using Array3d = Array3<double>;
using Array4d = Array4<double>;
using Array6d = Array6<double>;

// type definitions of complex
using Complex = std::complex<double>;

}  // namespace holon

#endif  // HOLON_COMMON_TYPES_HPP_
