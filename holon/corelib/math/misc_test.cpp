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

#include "holon/corelib/math/misc.hpp"

#include <limits>

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

TEST_CASE("sign function", "[math][misc][sgn]") {
  CHECK(sgn(10) == 1);
  CHECK(sgn(-10) == -1);
  CHECK(sgn(0) == 0);
}

TEST_CASE("square value", "[math][misc][square]") {
  CHECK(square(5) == 25);
  CHECK(square(1.1) == Approx(1.21));
  CHECK(square(-1) == 1);
}

TEST_CASE("limit returns saturated value with lower and upper boundaries",
          "[math][misc][limit]") {
  CHECK(limit(0, -1, 1) == 0);
  CHECK(limit(0.0, 0.5, 1.0) == 0.5);
  CHECK(limit(0.0, -1.0, -0.5) == -0.5);
}

TEST_CASE("bound returns saturated value with two boundaries",
          "[math][misc][bound]") {
  CHECK(bound(0, -1, 1) == 0);
  CHECK(bound(0.0, 0.5, 1.0) == 0.5);
  CHECK(bound(0.0, -1.0, -0.5) == -0.5);

  CHECK(bound(0.0, 1.0, 0.5) == 0.5);
  CHECK(bound(0.0, -0.5, -1.0) == -0.5);
}

TEST_CASE("is_tiny returns true if the given value is tiny enough",
          "[math][misc][is_tiny]") {
  INFO(std::numeric_limits<double>::min());
  INFO(std::numeric_limits<double>::epsilon());
  CHECK(is_tiny(0.0));
  CHECK(is_tiny(std::numeric_limits<double>::min()));
  // CHECK(is_tiny(0));
}

TEST_CASE("is_positive returns true if the given value is positive",
          "[math][misc][is_positive]") {
  SECTION("int type") {
    CHECK(is_positive(1));
    CHECK_FALSE(is_positive(-1));
    CHECK_FALSE(is_positive(0));
  }
  SECTION("double type") {
    CHECK(is_positive(1.0));
    CHECK_FALSE(is_positive(-1.0));
    CHECK_FALSE(is_positive(0.0));
  }
}

TEST_CASE("is_negative returns true if the given value is negative",
          "[math][misc][is_negative]") {
  SECTION("int type") {
    CHECK(is_negative(-1));
    CHECK_FALSE(is_negative(1));
    CHECK_FALSE(is_negative(0));
  }
  SECTION("double type") {
    CHECK(is_negative(-1.0));
    CHECK_FALSE(is_negative(1.0));
    CHECK_FALSE(is_negative(0.0));
  }
}

}  // namespace
}  // namespace holon
