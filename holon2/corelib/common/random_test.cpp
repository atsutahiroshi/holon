/* random - Random number generator
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

#include "holon2/corelib/common/random.hpp"

#include <vector>
#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

TEST_CASE("random: generate pseudo-random number with double type",
          "[Random]") {
  std::vector<double> history;
  Random<double> rnd;

  for (auto i = 0; i < 10; ++i) {
    auto n = rnd();
    REQUIRE_THAT(history, !Catch::Matchers::VectorContains(n));
    history.push_back(n);
  }
}

TEST_CASE("random: exact the same number is generated given the same seed",
          "[Random]") {
  Random<double> rnd1({0});
  Random<double> rnd2({0});

  for (auto i = 0; i < 10; ++i) {
    REQUIRE(rnd1() == rnd2());
  }
}

TEST_CASE("random: limit the generating number within a specific range",
          "[Random]") {
  Random<double> rnd(0, 0.1);

  for (auto i = 0; i < 10; ++i) {
    REQUIRE(rnd() > 0);
    REQUIRE(rnd() < 0.1);
  }
}

}  // namespace
}  // namespace holon
