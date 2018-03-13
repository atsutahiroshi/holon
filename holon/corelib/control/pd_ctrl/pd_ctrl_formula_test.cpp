/* pd_ctrl_formula - Helper functions for simple PD control class
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

#include "holon/corelib/control/pd_ctrl/pd_ctrl_formula.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

using pd_ctrl_formula::computeDesForce;

namespace data_force {

template <typename params>
struct testcase_t {
  static constexpr typename params::type xd = params::xd;
  static constexpr typename params::type vd = params::vd;
  static constexpr typename params::type k = params::k;
  static constexpr typename params::type c = params::c;
  typename params::type x, v;
  typename params::type expected_force;
};

// case 1: xd = 1, vd = 0, k = 1, c = 1
template <typename T>
struct params1 {
  using type = T;
  static constexpr type xd = type{1};
  static constexpr type vd = type{0};
  static constexpr type k = type{1};
  static constexpr type c = type{1};
};
testcase_t<params1<double>> testcases1[] = {
    {0, 0, 1}, {1, 0, 0}, {2, 1, -2}, {-1, -1, 3}};

// case 2: xd = -1, vd = 1, k = 1, c = 1
template <typename T>
struct params2 {
  using type = T;
  static constexpr type xd = type{-1};
  static constexpr type vd = type{1};
  static constexpr type k = type{1};
  static constexpr type c = type{1};
};
testcase_t<params2<double>> testcases2[] = {
    {0, 0, 0}, {1, 1, -2}, {-1, 1, 0}, {2, -1, -1}};

// case 3: xd = 1, vd = 1, k = 2, c = 1
template <typename T>
struct params3 {
  using type = T;
  static constexpr type xd = type{1};
  static constexpr type vd = type{1};
  static constexpr type k = type{2};
  static constexpr type c = type{1};
};
testcase_t<params3<double>> testcases3[] = {
    {0, 0, 3}, {1, -1, 2}, {-1, 1, 4}, {2, 2, -3}};

// case 3: xd = 0, vd = -1, k = 1, c = 2
template <typename T>
struct params4 {
  using type = T;
  static constexpr type xd = type{0};
  static constexpr type vd = type{-1};
  static constexpr type k = type{1};
  static constexpr type c = type{2};
};
testcase_t<params4<double>> testcases4[] = {
    {0, 0, -2}, {1, 1, -5}, {-1, 1, -3}, {1, -1, -1}};

}  // namespace data_force

template <typename T, typename Testcase>
void CheckDesiredForce(const Testcase& testcases) {
  for (const auto& tc : testcases) {
    auto f =
        computeDesForce(T{tc.x}, T{tc.v}, T{tc.xd}, T{tc.vd}, T{tc.k}, T{tc.c});
    CHECK(f == T{tc.expected_force});
  }
}

TEST_CASE("Check if desired force computed correctly in PdCtrl (case 1)",
          "[PdCtrl][force][case1]") {
  SECTION("with double type") {
    CheckDesiredForce<double>(data_force::testcases1);
  }
  SECTION("with Vec3D type") {
    CheckDesiredForce<Vec3D>(data_force::testcases1);
  }
}

TEST_CASE("Check if desired force computed correctly in PdCtrl (case 2)",
          "[PdCtrl][force][case2]") {
  SECTION("with double type") {
    CheckDesiredForce<double>(data_force::testcases2);
  }
  SECTION("with Vec3D type") {
    CheckDesiredForce<Vec3D>(data_force::testcases2);
  }
}

TEST_CASE("Check if desired force computed correctly in PdCtrl (case 3)",
          "[PdCtrl][force][case3]") {
  SECTION("with double type") {
    CheckDesiredForce<double>(data_force::testcases3);
  }
  SECTION("with Vec3D type") {
    CheckDesiredForce<Vec3D>(data_force::testcases3);
  }
}

TEST_CASE("Check if desired force computed correctly in PdCtrl (case 4)",
          "[PdCtrl][force][case4]") {
  SECTION("with double type") {
    CheckDesiredForce<double>(data_force::testcases4);
  }
  SECTION("with Vec3D type") {
    CheckDesiredForce<Vec3D>(data_force::testcases4);
  }
}

}  // namespace
}  // namespace holon
