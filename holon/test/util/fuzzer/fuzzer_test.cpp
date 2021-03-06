/* fuzzer - simple fuzzer
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

#include "holon/test/util/fuzzer/fuzzer.hpp"

#include <vector>
#include "catch.hpp"
#include "holon/test/util/catch/custom_matchers.hpp"

namespace holon {
namespace {

using Catch::Matchers::Equals;
using Catch::Matchers::VectorContains;

TEST_CASE("outputs random values", "[test][util][fuzzer]") {
  std::vector<double> outdated_values;
  Fuzzer fuzz;

  for (auto i = 0; i < 10; ++i) {
    double n = fuzz.get();
    REQUIRE_THAT(outdated_values, !VectorContains(n));
    outdated_values.push_back(n);
  }
}

TEST_CASE("outputs same values with a specific seed", "[test][util][fuzzer]") {
  Fuzzer fuzz1({0});
  Fuzzer fuzz2({0});

  for (auto i = 0; i < 10; ++i) {
    REQUIRE(fuzz1.get() == fuzz2.get());
  }
}

TEST_CASE("make uniform distribution within some range",
          "[test][util][fuzzer]") {
  Fuzzer fuzz(0, 10);

  for (auto i = 0; i < 10; ++i) {
    double v = fuzz.get();
    REQUIRE(v > 0);
    REQUIRE(v < 10);
  }
}

TEST_CASE("make uniform distribution with a specific seed",
          "[test][util][fuzzer]") {
  Fuzzer fuzz1({0}, 0, 10);
  Fuzzer fuzz2({0}, 0, 10);

  for (auto i = 0; i < 10; ++i) {
    double v1 = fuzz1.get();
    double v2 = fuzz2.get();
    REQUIRE(v1 == v2);
    REQUIRE(v1 > 0);
    REQUIRE(v1 < 10);
    REQUIRE(v2 > 0);
    REQUIRE(v2 < 10);
  }
}

TEST_CASE("calling by ()operator", "[test][util][fuzzer]") {
  Fuzzer fuzz;
  std::vector<double> outdated_values;

  for (auto i = 0; i < 10; ++i) {
    double n = fuzz();
    REQUIRE_THAT(outdated_values, !VectorContains(n));
    outdated_values.push_back(n);
  }
}

TEST_CASE("randomize elements in zVec3D", "[test][util][fuzzer]") {
  Fuzzer fuzz;

  for (auto i = 0; i < 10; ++i) {
    zVec3D v1, v2;
    fuzz.randomize(v1);
    fuzz.randomize(v2);
    REQUIRE_THAT(v1, !Equals(v2));
  }
}

TEST_CASE("randomize elements in Vec3D", "[test][util][fuzzer]") {
  Fuzzer fuzz;

  for (auto i = 0; i < 10; ++i) {
    Vec3D v1, v2;
    fuzz.randomize(v1);
    fuzz.randomize(v2);
    REQUIRE_THAT(v1, !Equals(v2));
  }
}

TEST_CASE("get randomized Vec3D", "[test][util][fuzzer]") {
  Fuzzer fuzz;

  for (auto i = 0; i < 10; ++i) {
    REQUIRE_THAT(fuzz.get<Vec3D>(), !Equals(fuzz.get<Vec3D>()));
  }
}
}  // namespace
}  // namespace holon
