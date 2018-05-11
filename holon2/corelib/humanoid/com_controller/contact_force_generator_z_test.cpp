/* contact_force_generator - Contact force generator class
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

#include "holon2/corelib/humanoid/com_controller/contact_force_generator.hpp"

#include "holon2/corelib/humanoid/const_defs.hpp"
#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

const double G = kGravAccel;
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

#define XI2(zd) (G / zd)
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
void checkContactForceZ_1(const testcase& testcases) {
  ComControllerData data;
  ContactForceGenerator cf(data);
  auto& params = data.get<2>();
  for (const auto& tc : testcases) {
    params.com_position[2] = tc.zd;
    params.q1[2] = tc.q1;
    params.q2[2] = tc.q2;
    data.get<0>().mass = tc.mass;
    auto fz = cf.calculateZ(tc.z, tc.v);
    CHECK(fz == Approx(tc.expected_fz));
  }
}

TEST_CASE("contact_force_generator: calculate vertical contact force: case 1",
          "[ComController][ContactForceGenerator]") {
  SECTION("overloaded function 1") { checkContactForceZ_1(testcases1); }
}

TEST_CASE("contact_force_generator: calculate vertical contact force: case 2",
          "[ComController][ContactForceGenerator]") {
  SECTION("overloaded function 1") { checkContactForceZ_1(testcases2); }
}

TEST_CASE("contact_force_generator: calculate vertical contact force: case 3",
          "[ComController][ContactForceGenerator]") {
  SECTION("overloaded function 1") { checkContactForceZ_1(testcases3); }
}

TEST_CASE("contact_force_generator: calculate vertical contact force: case 4",
          "[ComController][ContactForceGenerator]") {
  // zEchoOff();
  SECTION("overloaded function 1") { checkContactForceZ_1(testcases4); }
  // zEchoOn();
}

}  // namespace
}  // namespace holon
