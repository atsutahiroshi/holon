/* misc - miscellanies related to mathematics.
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

#include "holon2/corelib/math/misc.hpp"

#include <limits>

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

TEST_CASE("misc: sign function", "[math][misc][sgn]") {
  CHECK(sgn(10) == 1);
  CHECK(sgn(-10) == -1);
  CHECK(sgn(0) == 0);
}

TEST_CASE("misc: compute squared value", "[math][misc][sqr]") {
  CHECK(sqr(5) == 25);
  CHECK(sqr(1.1) == Approx(1.21));
  CHECK(sqr(-1) == 1);
}

TEST_CASE("misc: limit returns saturated value with lower and upper boundaries",
          "[math][misc][limit]") {
  CHECK(limit(0, -1, 1) == 0);
  CHECK(limit(0.0, 0.5, 1.0) == 0.5);
  CHECK(limit(0.0, -1.0, -0.5) == -0.5);
}

TEST_CASE("misc: bound returns saturated value with two boundaries",
          "[math][misc][bound]") {
  CHECK(bound(0, -1, 1) == 0);
  CHECK(bound(0.0, 0.5, 1.0) == 0.5);
  CHECK(bound(0.0, -1.0, -0.5) == -0.5);

  CHECK(bound(0.0, 1.0, 0.5) == 0.5);
  CHECK(bound(0.0, -0.5, -1.0) == -0.5);
}

TEST_CASE("misc: isTiny returns true if the given value is tiny enough",
          "[math][misc][isTiny]") {
  INFO(std::numeric_limits<double>::min());
  INFO(std::numeric_limits<double>::epsilon());
  CHECK(isTiny(0.0));
  CHECK(isTiny(std::numeric_limits<double>::min()));
  // CHECK(isTiny(0));
}

TEST_CASE("misc: isPositive returns true if the given value is positive",
          "[math][misc][isPositive]") {
  SECTION("int type") {
    CHECK(isPositive(1));
    CHECK_FALSE(isPositive(-1));
    CHECK_FALSE(isPositive(0));
  }
  SECTION("double type") {
    CHECK(isPositive(1.0));
    CHECK_FALSE(isPositive(-1.0));
    CHECK_FALSE(isPositive(0.0));
  }
}

TEST_CASE("misc: isNegative returns true if the given value is negative",
          "[math][misc][isNegative]") {
  SECTION("int type") {
    CHECK(isNegative(-1));
    CHECK_FALSE(isNegative(1));
    CHECK_FALSE(isNegative(0));
  }
  SECTION("double type") {
    CHECK(isNegative(-1.0));
    CHECK_FALSE(isNegative(1.0));
    CHECK_FALSE(isNegative(0.0));
  }
}

}  // namespace
}  // namespace holon
