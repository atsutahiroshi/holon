/* utility - Utility functions and classes for template meta programming
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of holon.
 *
 * Holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HOLON_COMMON_UTILITY_HPP_
#define HOLON_COMMON_UTILITY_HPP_

#include <type_traits>
#include <utility>
#include "holon2/corelib/common/detail/utility_detail.hpp"

namespace holon {

template <std::size_t... Ints>
struct IndexSeq {
  static constexpr std::size_t size() { return sizeof...(Ints); }

  template <std::size_t I>
  static constexpr std::size_t get() {
    static_assert(I >= 0, "IndexSeq::get<I>(): 'I' must not be negative");
    static_assert(I < sizeof...(Ints), "IndexSeq::get<I>(): out of range");
    return ints[I];
  }

 private:
  static constexpr std::size_t ints[] = {Ints...};
};

template <std::size_t N>
using makeIndexSeq =
    typename utility_detail::makeIndexSeq_impl<N, IndexSeq<>>::type;

}  // namespace holon

#endif  // HOLON_COMMON_UTILITY_HPP_
