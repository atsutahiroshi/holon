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

#include <cmath>
#include "holon/corelib/humanoid/biped_foot_model.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace com_guided_ctrl_formula {
namespace {

using std::sqrt;

namespace check_omega {

struct testcase_t {
  double q1, q2;
  double zeta;
  double expected_omega;
};
testcase_t testcases1[] = {{1, 1, 1, 1},
                           {0.5, 1, 1, sqrt(0.5)},
                           {2, 1, 0.5, 0.5 * sqrt(2)},
                           {0.1, 0.5, 0.1, 0.1 * sqrt(0.05)}};
testcase_t testcases2[] = {{0, 1, 1, 0},  {1, 0, 1, 0},  {1, 1, 0, 0},
                           {-1, 1, 1, 0}, {1, -1, 1, 0}, {1, 1, -1, 0}};

TEST_CASE("Check frequency of oscillation",
          "[com_guided_ctrl_formula][frequency]") {
  for (const auto& tc : testcases1) {
    auto omega = frequency(tc.q1, tc.q2, tc.zeta);
    CHECK(omega == Approx(tc.expected_omega));
  }
}

TEST_CASE("Check if frequency of oscillation is positive",
          "[com_guided_ctrl_formula][frequency][exception]") {
  zEchoOff();
  for (const auto& tc : testcases2) {
    auto omega = frequency(tc.q1, tc.q2, tc.zeta);
    CHECK(omega == Approx(tc.expected_omega));
  }
  zEchoOn();
}

}  // namespace check_omega

namespace check_period {

struct testcase_t {
  double q1, q2, zeta;
  double expected_T;
};
testcase_t testcases1[] = {{1, 1, 1, zPIx2},
                           {0.5, 1, 1, zPIx2 / sqrt(0.5)},
                           {2, 1, 0.5, zPIx2 / (0.5 * sqrt(2))},
                           {0.1, 0.5, 0.1, zPIx2 / (0.1 * sqrt(0.05))}};
testcase_t testcases2[] = {{0, 1, 1, 0},  {1, 0, 1, 0},  {1, 1, 0, 0},
                           {-1, 1, 1, 0}, {1, -1, 1, 0}, {1, 1, -1, 0}};

TEST_CASE("Check period of oscillation", "[com_guided_ctrl_formula]][period]") {
  for (const auto& tc : testcases1) {
    auto T = period(tc.q1, tc.q2, tc.zeta);
    CHECK(T == Approx(tc.expected_T));
  }
}

TEST_CASE("Check period when given frequency is zero",
          "[com_guided_ctrl_formula][period][exception]") {
  zEchoOff();
  for (const auto& tc : testcases2) {
    auto T = period(tc.q1, tc.q2, tc.zeta);
    CHECK(T == Approx(tc.expected_T));
  }
  zEchoOn();
}

}  // namespace check_period

namespace check_complex_zmp_y {

struct testcase_t {
  double yz, yd;
  double vy, q1, q2, zeta;
  Complex expected_pz;
};
testcase_t testcases1[] = {{0, 0, 0, 1, 1, 1, {0, 0}},
                           {1, 0, 1, 1, 1, 1, {1, -2}},
                           {0, 1, 1, 2, 2, 0.5, {-1, -5}},
                           {1, 0.5, 0.1, 0.4, 0.9, 1, {0.5, -0.136 / 0.6}}};

TEST_CASE("Check complex number defined from ZMP along y-axis",
          "[com_guided_ctrl_formula][complex_zmp_y]") {
  SECTION("oveloaded function 1") {
    for (const auto& tc : testcases1) {
      auto pz = complex_zmp_y(tc.yz, tc.vy, tc.yd, tc.q1, tc.q2, tc.zeta);
      CHECK(pz.real() == Approx(tc.expected_pz.real()));
      CHECK(pz.imag() == Approx(tc.expected_pz.imag()));
    }
  }
}

}  // namespace check_complex_zmp_y

namespace check_complex_inner_edge {

struct testcase_t {
  double yin, yd;
  Complex pz;
  BipedFootType type;
  Complex expected_p0;
};
testcase_t testcases1[] = {{0, 0, {1, 0}, BipedFootType::left, {0, -1}},
                           {1, 1, {-1, 1}, BipedFootType::left, {0, -sqrt(2)}},
                           {1, 0, {3, 4}, BipedFootType::left, {1, -sqrt(24)}},
                           {0, 1, {1, 2}, BipedFootType::left, {-1, -2}}};
testcase_t testcases2[] = {{0, 0, {1, 0}, BipedFootType::right, {0, 1}},
                           {1, 1, {-1, 1}, BipedFootType::right, {0, sqrt(2)}},
                           {1, 0, {3, 4}, BipedFootType::right, {1, sqrt(24)}},
                           {0, 1, {1, 2}, BipedFootType::right, {-1, 2}}};

template <typename testcase_t>
void CheckCompleInnerEdge_1(const testcase_t& testcases) {
  SECTION("oveloaded function 1") {
    for (const auto& tc : testcases) {
      auto p0 = complex_inner_edge(tc.yin, tc.yd, tc.pz, tc.type);
      CHECK(p0.real() == Approx(tc.expected_p0.real()));
      CHECK(p0.imag() == Approx(tc.expected_p0.imag()));
    }
  }
}

TEST_CASE("Case 1: complex number defined from inner edge",
          "[com_guided_ctrl_formula][complex_inner_edge]") {
  CheckCompleInnerEdge_1(testcases1);
}

TEST_CASE("Case 2: complex number defined from inner edge",
          "[com_guided_ctrl_formula][complex_inner_edge]") {
  CheckCompleInnerEdge_1(testcases2);
}

}  // namespace check_complex_inner_edge

namespace check_phase {

template <typename params>
struct testcase_t {
  static constexpr double yd = params::yd;
  static constexpr double yin = params::yin;
  static constexpr double q1 = params::q1;
  static constexpr double q2 = params::q2;
  static constexpr double zeta = params::zeta;
  static constexpr BipedFootType type = params::type;
  double yz, vy;
  double expected_phase;
};

// case 1: yd = 0, yin = 0, q1 = 1, q2 = 1, zeta = 1, left foot
struct params1 {
  static constexpr double yd = 0;
  static constexpr double yin = 0;
  static constexpr double q1 = 1;
  static constexpr double q2 = 1;
  static constexpr double zeta = 1;
  static constexpr BipedFootType type = BipedFootType::left;
};
testcase_t<params1> testcases1[] = {
    {0, 1, 0},  {0.5, 0.5, 0.25}, {1, 0, 0.5}, {0.5, -0.5, 0.75},
    {0, -1, 1}, {-0.5, -0.5, 0},  {-1, 0, 0},  {-0.5, 0.5, 0}};

// case 2: yd = 0, yin = 0, q1 = 1, q2 = 1, zeta = 1, right foot
struct params2 {
  static constexpr double yd = 0;
  static constexpr double yin = 0;
  static constexpr double q1 = 1;
  static constexpr double q2 = 1;
  static constexpr double zeta = 1;
  static constexpr BipedFootType type = BipedFootType::right;
};
testcase_t<params2> testcases2[] = {
    {0, 1, 1},  {0.5, 0.5, 0},      {1, 0, 0},    {0.5, -0.5, 0},
    {0, -1, 0}, {-0.5, -0.5, 0.25}, {-1, 0, 0.5}, {-0.5, 0.5, 0.75}};

// case 3: yd = 0, yin = 0.1, q1 = 1, q2 = 1, zeta = 1, left foot
struct params3 {
  static constexpr double yd = 0;
  static constexpr double yin = 0.1;
  static constexpr double q1 = 1;
  static constexpr double q2 = 1;
  static constexpr double zeta = 1;
  static constexpr BipedFootType type = BipedFootType::left;
};
testcase_t<params3> testcases3[] = {
    {0, 1, 0},  {0.5, 0.5, 0.25}, {1, 0, 0.5}, {0.5, -0.5, 0.75},
    {0, -1, 1}, {-0.5, -0.5, 0},  {-1, 0, 0},  {-0.5, 0.5, 0}};

// case 4: yd = 0, yin = -0.1, q1 = 1, q2 = 1, zeta = 1, right foot
struct params4 {
  static constexpr double yd = 0;
  static constexpr double yin = -0.1;
  static constexpr double q1 = 1;
  static constexpr double q2 = 1;
  static constexpr double zeta = 1;
  static constexpr BipedFootType type = BipedFootType::right;
};
testcase_t<params4> testcases4[] = {
    {0, 1, 1},  {0.5, 0.5, 0},      {1, 0, 0},    {0.5, -0.5, 0},
    {0, -1, 0}, {-0.5, -0.5, 0.25}, {-1, 0, 0.5}, {-0.5, 0.5, 0.75}};

void CheckPhase_common(const double phase, const double expected_phase) {
  if (expected_phase == 0.25) {
    CHECK(phase > 0.0);
    CHECK(phase < 0.5);
  } else if (expected_phase == 0.75) {
    CHECK(phase > 0.75);
    CHECK(phase < 1.0);
  } else {
    CHECK(phase == Approx(expected_phase));
  }
}

template <typename testcase_t>
void CheckPhase_1(const testcase_t& testcases) {
  SECTION("overloaded function 1") {
    for (const auto& tc : testcases) {
      auto phase =
          phase_y(tc.yz, tc.vy, tc.yd, tc.yin, tc.q1, tc.q2, tc.zeta, tc.type);
      CAPTURE(tc.yz);
      CAPTURE(tc.vy);
      CheckPhase_common(phase, tc.expected_phase);
    }
  }
}

TEST_CASE("Case 1: oscillation phase defined along y-axis",
          "[com_guided_ctrl_formula][phase_y]") {
  CheckPhase_1(testcases1);
}

TEST_CASE("Case 2: oscillation phase defined along y-axis",
          "[com_guided_ctrl_formula][phase_y]") {
  CheckPhase_1(testcases2);
}

TEST_CASE("Case 3: oscillation phase defined along y-axis",
          "[com_guided_ctrl_formula][phase_y]") {
  CheckPhase_1(testcases3);
}

TEST_CASE("Case 4: oscillation phase defined along y-axis",
          "[com_guided_ctrl_formula][phase_y]") {
  CheckPhase_1(testcases4);
}

}  // namespace check_phase

}  // namespace
}  // namespace com_guided_ctrl_formula
}  // namespace holon
