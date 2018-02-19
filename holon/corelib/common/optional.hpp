/* optional - ensure compatibility with optional
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

#ifndef HOLON_COMMON_OPTIONAL_HPP_
#define HOLON_COMMON_OPTIONAL_HPP_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include "third_party/optional-lite/optional.hpp"
#pragma GCC diagnostic pop

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

#endif  // HOLON_COMMON_OPTIONAL_HPP_
