/* zmp_manipulator - ZMP manipulator class
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

#include "holon2/corelib/humanoid/com_controller/zmp_manipulator.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

template <typename params>
struct testcase_t {
  static constexpr typename params::type yd = params::yd;
  static constexpr typename params::type q1 = params::q1;
  static constexpr typename params::type q2 = params::q2;
  static constexpr typename params::type rho = params::rho;
  static constexpr typename params::type dist = params::dist;
  static constexpr typename params::type kr = params::kr;
  static constexpr typename params::type zeta = params::zeta;
  double y, v;
  double expected_yz;
};

// case 1: yd = 0, q1 = 1, q2 = 1, zeta = 1
template <typename T = double>
struct params1 {
  using type = T;
  static constexpr type yd = 0;
  static constexpr type q1 = 1;
  static constexpr type q2 = 1;
  static constexpr type rho = 0;
  static constexpr type dist = 0;
  static constexpr type kr = 1;
  static constexpr type zeta = 1;
};
testcase_t<params1<>> testcases1[] = {
    {0, 0, 0}, {1, 0, 2}, {3, -1, 4}, {0, -2, -4}, {-2, 2, 0}};

// case 2: yd = 0, q1 = 1, q2 = 0.5, zeta = 1
template <typename T = double>
struct params2 {
  using type = T;
  static constexpr type yd = 0;
  static constexpr type q1 = 1;
  static constexpr type q2 = 0.5;
  static constexpr type rho = 0;
  static constexpr type dist = 0;
  static constexpr type kr = 1;
  static constexpr type zeta = 1;
};
testcase_t<params2<>> testcases2[] = {
    {0, 0, 0}, {1, 0, 1.5}, {3, -1, 3}, {0, -2, -3}, {-2, 2, 0}};

// case 3: yd = 0, q1 = 1.2, q2 = 0.8, zeta = 1
template <typename T = double>
struct params3 {
  using type = T;
  static constexpr type yd = 0;
  static constexpr type q1 = 1.2;
  static constexpr type q2 = 0.8;
  static constexpr type rho = 0;
  static constexpr type dist = 0;
  static constexpr type kr = 1;
  static constexpr type zeta = 1;
};
testcase_t<params3<>> testcases3[] = {
    {0, 0, 0}, {1, 0, 1.96}, {3, -1, 3.88}, {0, -2, -4}, {-2, 2, 0.08}};

// case 4: yd = 0.5, q1 = 1, q2 = 1, zeta = 1
template <typename T = double>
struct params4 {
  using type = T;
  static constexpr type yd = 0.5;
  static constexpr type q1 = 1;
  static constexpr type q2 = 1;
  static constexpr type rho = 0;
  static constexpr type dist = 0;
  static constexpr type kr = 1;
  static constexpr type zeta = 1;
};
testcase_t<params4<>> testcases4[] = {
    {0, 0, -0.5}, {1, 0, 1.5}, {3, -1, 3.5}, {0, -2, -4.5}, {-2, 3, 1.5}};

// case 5: yd = -1, q1 = 1, q2 = 1.5, zeta = 1
template <typename T = double>
struct params5 {
  using type = T;
  static constexpr type yd = -1;
  static constexpr type q1 = 1;
  static constexpr type q2 = 1.5;
  static constexpr type rho = 0;
  static constexpr type dist = 0;
  static constexpr type kr = 1;
  static constexpr type zeta = 1;
};
testcase_t<params5<>> testcases5[] = {
    {0, 0, 1.5}, {1, 0, 4}, {3, -1, 6.5}, {0, -2, -3.5}, {-2, 3, 4}};

// case 6: yd = 0.5, q1 = 1, q2 = 1.5, zeta = sqrt(2)
template <typename T = double>
struct params6 {
  using type = T;
  static constexpr type yd = 0.5;
  static constexpr type q1 = 1;
  static constexpr type q2 = 1.5;
  static constexpr type rho = 0;
  static constexpr type dist = 0;
  static constexpr type kr = 1;
  static constexpr type zeta = 1.41421356237;  // sqrt(2)
};
testcase_t<params6<>> testcases6[] = {
    {0, 0, -0.75},
    {1, 0, 1.75},
    {3, -2, 2.5 * (3 - sqrt(2)) - 0.75},
    {0, -1, -1.25 * sqrt(2) - 0.75},
    {-2, 3, 2.5 * (-2 + 1.5 * sqrt(2)) - 0.75}};

// case 7: yd = 1, q1 = 1, q2 = 1, y = 0, v = 0
struct testcase7_t {
  static constexpr double yd = 1;
  static constexpr double q1 = 1;
  static constexpr double q2 = 1;
  static constexpr double rho = 0;
  static constexpr double dist = 0;
  static constexpr double kr = 1;
  static constexpr double y = 0;
  static constexpr double v = 0;
  double zeta;
  double expected_yz;
};
testcase7_t testcases7[] = {{0, 0}, {-1, 0}};

// case 8: yd = 0.5, q1 = 1, q2 = 1, zeta = 1, rho =1, dist = 0
template <typename T = double>
struct params8 {
  using type = T;
  static constexpr type yd = 0.5;
  static constexpr type q1 = 1;
  static constexpr type q2 = 1;
  static constexpr type rho = 1;
  static constexpr type dist = 0;
  static constexpr type kr = 1;
  static constexpr type zeta = 1;
};
testcase_t<params8<>> testcases8[] = {
    {0, 0, -0.5}, {1, 0, 1.5}, {3, -1, 3.5}, {0, -2, -4.5}, {-2, 3, 1.5}};

#if 0
// case __:
template <typename T = double>
struct params__ {
  using type = T;
  static constexpr type yd = ;
  static constexpr type q1 = ;
  static constexpr type q2 = ;
  static constexpr type rho = 0;
  static constexpr type dist = 0;
  static constexpr type kr = 1;
  static constexpr type zeta = ;
};
testcase_t<params__<>> testcases__[] = {};
#endif

template <typename Testcase>
void checkCalculateY_1(const Testcase& testcases) {
  ComControllerData data;
  ZmpManipulator zmp(data);
  auto& params = data.get<2>();
  for (const auto& tc : testcases) {
    params.com_position[1] = tc.yd;
    params.q1[1] = tc.q1;
    params.q2[1] = tc.q2;
    params.rho = tc.rho;
    params.dist = tc.dist;
    params.kr = tc.kr;
    auto yz = zmp.calculateY(tc.y, tc.v, tc.zeta);
    CHECK(yz == Approx(tc.expected_yz));
  }
}

TEST_CASE(
    "zmp_manipulator: calculate y-axis ZMP with COM-guided control: case 1",
    "[ComController][ZmpManipulator]") {
  SECTION("overloaded function 1") { checkCalculateY_1(testcases1); }
}

TEST_CASE(
    "zmp_manipulator: calculate y-axis ZMP with COM-guided control: case 2",
    "[ComController][ZmpManipulator]") {
  SECTION("overloaded function 1") { checkCalculateY_1(testcases2); }
}

TEST_CASE(
    "zmp_manipulator: calculate y-axis ZMP with COM-guided control: case 3",
    "[ComController][ZmpManipulator]") {
  SECTION("overloaded function 1") { checkCalculateY_1(testcases3); }
}

TEST_CASE(
    "zmp_manipulator: calculate y-axis ZMP with COM-guided control: case 4",
    "[ComController][ZmpManipulator]") {
  SECTION("overloaded function 1") { checkCalculateY_1(testcases4); }
}

TEST_CASE(
    "zmp_manipulator: calculate y-axis ZMP with COM-guided control: case 5",
    "[ComController][ZmpManipulator]") {
  SECTION("overloaded function 1") { checkCalculateY_1(testcases5); }
}

TEST_CASE(
    "zmp_manipulator: calculate y-axis ZMP with COM-guided control: case 6",
    "[ComController][ZmpManipulator]") {
  SECTION("overloaded function 1") { checkCalculateY_1(testcases6); }
}

TEST_CASE(
    "zmp_manipulator: calculate y-axis ZMP with COM-guided control: case 7",
    "[ComController][ZmpManipulator]") {
  // zEchoOff();
  SECTION("overloaded function 1") { checkCalculateY_1(testcases7); }
  // zEchoOn();
}

TEST_CASE(
    "zmp_manipulator: calculate y-axis ZMP with COM-guided control: case 8",
    "[ComController][ZmpManipulator]") {
  SECTION("overloaded function 1") { checkCalculateY_1(testcases8); }
}

TEST_CASE(
    "zmp_manipulator: calculate y-axis ZMP with COM-guided control: case 0",
    "[ComController][ZmpManipulator]") {}

}  // namespace
}  // namespace holon
