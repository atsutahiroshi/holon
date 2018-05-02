/* optional - type aliases of optional-lite
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

#ifndef HOLON_COMMON_OPTIONAL_HPP_
#define HOLON_COMMON_OPTIONAL_HPP_

#include <string>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include "third_party/optional-lite/optional.hpp"
#pragma GCC diagnostic pop
#include "third_party/catch/catch.hpp"

namespace holon {

using nonstd::optional;
using nonstd::bad_optional_access;

using nonstd::nullopt;
using nonstd::nullopt_t;
using nonstd::in_place;
using nonstd::in_place_type;
using nonstd::in_place_index;
using nonstd::in_place_t;

using nonstd::operator==;
using nonstd::operator!=;
using nonstd::operator<;
using nonstd::operator<=;
using nonstd::operator>;
using nonstd::operator>=;
using nonstd::make_optional;
using nonstd::swap;

}  // holon

namespace Catch {

template <typename T>
struct StringMaker<holon::optional<T>> {
  static std::string convert(const holon::optional<T>& v) {
    if (v.has_value())
      return ::Catch::Detail::stringify(v.value());
    else
      return std::string("nullopt");
  }
};

}  // namespace Catch

#endif  // HOLON_COMMON_OPTIONAL_HPP_
