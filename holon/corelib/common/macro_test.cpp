/* macro - Macro header
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

#include "holon/corelib/common/macro.hpp"

#include "catch.hpp"

namespace holon {
namespace {

#define VERY_LONG_SEQ                                                         \
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,  \
      22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, \
      40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, \
      58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75

TEST_CASE("Check HOLON_VA_ARGS_NUM", "[macro][HOLON_VA_ARGS_NUM]") {
  // CHECK(HOLON_VA_ARGS_NUM() == 0);
  CHECK(HOLON_VA_ARGS_NUM(1) == 1);
  CHECK(HOLON_VA_ARGS_NUM(1, 2, 3) == 3);
  CHECK(HOLON_VA_ARGS_NUM(VERY_LONG_SEQ) == 75);
}

TEST_CASE("Check HOLON_VA_ARGS_GET_FIRST", "[macro][HOLON_VA_ARGS_GET_FIRST]") {
  CHECK(HOLON_VA_ARGS_GET_FIRST("first", "second", "third") == "first");
}

}  // namespace
}  // namespace holon
