/* com_guided_ctrl_formula - Formulae for COM-guided control
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

#include "holon/corelib/humanoid/com_ctrl/com_guided_ctrl_formula.hpp"

#include <cure/cure_misc.h>
#include <roki/rk_g.h>

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace com_guided_ctrl_formula {
namespace {

const double G = RK_G;
const double SQRT_G = sqrt(G);

template <typename params>
struct testcase_t {
  static constexpr typename params::type zd = params::zd;
  static constexpr typename params::type q1 = params::q1;
  static constexpr typename params::type q2 = params::q2;
  static constexpr typename params::type mass = params::mass;
  double z, v;
  double expected_fz;
};

// case 1: zd = 1, q1 = 1, q2 = 1, m = 1
template <typename T = double>
struct params1 {
  using type = T;
  static constexpr type zd = 1;
  static constexpr type q1 = 1;
  static constexpr type q2 = 1;
  static constexpr type mass = 1;
};
testcase_t<params1<>> testcases1[] = {{1, 0, G},
                                      {0.5, 0.5, 1.5 * G - SQRT_G},
                                      {1.5, -1, 0.5 * G + 2. * SQRT_G},
                                      {0.8, -0.5, 1.2 * G + SQRT_G}};

#define XI2(zd) (RK_G / zd)
#define XI(zd) sqrt(XI2(zd))
// case 2: zd = 0.42, q1 = 0.5, q2 = 1, m = 1
template <typename T = double>
struct params2 {
  using type = T;
  static constexpr type zd = 0.42;
  static constexpr type q1 = 0.5;
  static constexpr type q2 = 1;
  static constexpr type mass = 1;
};
testcase_t<params2<>> testcases2[] = {
    {0.46, 0, 20. * G / 21},
    {0.42, -0.5, 0.75 * XI(0.42) + G},
    {0.4, 0.1, 0.01 * XI2(0.42) - 0.15 * XI(0.42) + G},
    {0.4, -0.1, 0.01 * XI2(0.42) + 0.15 * XI(0.42) + G}};

// case 3: zd = 0.42, q1 = 1, q2 = 0.5, m = 1.5
template <typename T = double>
struct params3 {
  using type = T;
  static constexpr type zd = 0.42;
  static constexpr type q1 = 1;
  static constexpr type q2 = 0.5;
  static constexpr type mass = 1.5;
};
testcase_t<params3<>> testcases3[] = {
    {0.46, 0, 30. * G / 21},
    {0.42, -0.5, 1.125 * XI(0.42) + 1.5 * G},
    {0.4, 0.1, 0.015 * XI2(0.42) - 0.225 * XI(0.42) + 1.5 * G},
    {0.4, -0.1, 0.015 * XI2(0.42) + 0.225 * XI(0.42) + 1.5 * G}};

// case 4: zd = 0.42, q1 = 1, q2 = 1, m = 1
template <typename T = double>
struct params4 {
  using type = T;
  static constexpr type zd = 0.42;
  static constexpr type q1 = 1;
  static constexpr type q2 = 1;
  static constexpr type mass = 1;
};
testcase_t<params4<>> testcases4[] = {{0.46, 1, 0}};

#if 0
// case __:
template <typename T = double>
struct params__ {
  using type = T;
  static constexpr type zd = ;
  static constexpr type q1 = ;
  static constexpr type q2 = ;
  static constexpr type mass = ;
};
testcase_t<params__<>> testcases__[] = {};
#endif

template <typename testcase>
void CheckReactionForce_1(const testcase& testcases) {
  for (const auto& tc : testcases) {
    auto fz =
        desired_reaction_force_z(tc.z, tc.v, tc.zd, tc.q1, tc.q2, tc.mass);
    CHECK(fz == Approx(tc.expected_fz));
  }
}

TEST_CASE("Case 1: desired vertical reaction force with COM-guided control",
          "[com_guided_ctrl_formula][desired_reaction_force_z]") {
  SECTION("overloaded function 1") { CheckReactionForce_1(testcases1); }
}

TEST_CASE("Case 2: desired vertical reaction force with COM-guided control",
          "[com_guided_ctrl_formula][desired_reaction_force_z]") {
  SECTION("overloaded function 1") { CheckReactionForce_1(testcases2); }
}

TEST_CASE("Case 3: desired vertical reaction force with COM-guided control",
          "[com_guided_ctrl_formula][desired_reaction_force_z]") {
  SECTION("overloaded function 1") { CheckReactionForce_1(testcases3); }
}

TEST_CASE("Case 4: desired vertical reaction force with COM-guided control",
          "[com_guided_ctrl_formula][desired_reaction_force_z][exception]") {
  zEchoOff();
  SECTION("overloaded function 1") { CheckReactionForce_1(testcases4); }
  zEchoOn();
}

}  // namespace
}  // namespace com_guided_ctrl_formula
}  // namespace holon
