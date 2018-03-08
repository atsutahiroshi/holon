/* zip - simple zip function
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

#ifndef HOLON_COMMON_ZIP_HPP_
#define HOLON_COMMON_ZIP_HPP_

#include <array>
#include <tuple>

namespace holon {

template <size_t N, typename... T>
auto zip(const std::array<T, N>&... arrays) -> std::array<std::tuple<T...>, N> {
  std::array<std::tuple<T...>, N> iter;
  for (std::size_t i = 0; i < N; ++i) {
    iter[i] = std::make_tuple(arrays[i]...);
  }
  return iter;
}

}  // namespace holon

#endif  // HOLON_COMMON_ZIP_HPP_
