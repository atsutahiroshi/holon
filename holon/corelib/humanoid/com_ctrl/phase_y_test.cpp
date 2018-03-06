/* phase_y - computation of phase along y-axis
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

#include "holon/corelib/humanoid/com_ctrl/phase_y.hpp"

#include <cmath>

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace phase_y {
namespace {

using std::sqrt;

namespace data_omega {

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

}  // namespace data_omega

TEST_CASE("Compute frequency of oscillation", "[phase_y][omega]") {
  for (const auto& tc : data_omega::testcases1) {
    auto omega = computeFrequency(tc.q1, tc.q2, tc.zeta);
    CHECK(omega == Approx(tc.expected_omega));
  }
}

TEST_CASE("Frequency of oscillation must be positive",
          "[phase_y][omega][exception]") {
  zEchoOff();
  for (const auto& tc : data_omega::testcases2) {
    auto omega = computeFrequency(tc.q1, tc.q2, tc.zeta);
    CHECK(omega == Approx(tc.expected_omega));
  }
  zEchoOn();
}

namespace data_pz {

struct testcase_t {
  double yz, yd;
  double vy, q1, q2, zeta;
  Complex expected_pz;
};
testcase_t testcases1[] = {{0, 0, 0, 1, 1, 1, {0, 0}},
                           {1, 0, 1, 1, 1, 1, {1, -2}},
                           {0, 1, 1, 2, 2, 0.5, {-1, -5}},
                           {1, 0.5, 0.1, 0.4, 0.9, 1, {0.5, -0.136 / 0.6}}};

}  // namespace data_pz

TEST_CASE("Compute complex number defined from ZMP", "[phase_y][pz]") {
  SECTION("oveloaded function 1") {
    for (const auto& tc : data_pz::testcases1) {
      auto pz = computeComplexZmp(tc.yz, tc.vy, tc.yd, tc.q1, tc.q2, tc.zeta);
      CHECK(pz.real() == Approx(tc.expected_pz.real()));
      CHECK(pz.imag() == Approx(tc.expected_pz.imag()));
    }
  }
  SECTION("oveloaded function 2") {
    for (const auto& tc : data_pz::testcases1) {
      Vec3D p_z = {0, tc.yz, 0};
      Vec3D v = {0, tc.vy, 0};
      Vec3D pd = {0, tc.yd, 1};
      auto pz = computeComplexZmp(p_z, v, pd, tc.q1, tc.q2, tc.zeta);
      CHECK(pz.real() == Approx(tc.expected_pz.real()));
      CHECK(pz.imag() == Approx(tc.expected_pz.imag()));
    }
  }
}

namespace data_p0 {

struct testcase_t {
  double yin, yd;
  Complex pz;
  int is_left;
  Complex expected_p0;
};
testcase_t testcases1[] = {{0, 0, {1, 0}, 1, {0, -1}},
                           {1, 1, {-1, 1}, 1, {0, -sqrt(2)}},
                           {1, 0, {3, 4}, 1, {1, -sqrt(24)}},
                           {0, 1, {1, 2}, 1, {-1, -2}}};
testcase_t testcases2[] = {{0, 0, {1, 0}, -1, {0, 1}},
                           {1, 1, {-1, 1}, -1, {0, sqrt(2)}},
                           {1, 0, {3, 4}, -1, {1, sqrt(24)}},
                           {0, 1, {1, 2}, -1, {-1, 2}}};

}  // namespace data_p0

template <typename testcase_t>
void check_p0_overloaded_funtion1(const testcase_t& testcases) {
  SECTION("oveloaded function 1") {
    for (const auto& tc : testcases) {
      auto p0 = computeComplexInnerEdge(tc.yin, tc.yd, tc.pz, tc.is_left);
      CHECK(p0.real() == Approx(tc.expected_p0.real()));
      CHECK(p0.imag() == Approx(tc.expected_p0.imag()));
    }
  }
}

template <typename testcase_t>
void check_p0_overloaded_funtion2(const testcase_t& testcases) {
  SECTION("oveloaded function 2") {
    for (const auto& tc : testcases) {
      Vec3D p_in = {0, tc.yin, 0};
      Vec3D pd = {0, tc.yd, 0};
      auto p0 = computeComplexInnerEdge(p_in, pd, tc.pz, tc.is_left);
      CHECK(p0.real() == Approx(tc.expected_p0.real()));
      CHECK(p0.imag() == Approx(tc.expected_p0.imag()));
    }
  }
}

TEST_CASE("Compute complex number defined from inner edge: case 1",
          "[phase_y][case1][p0]") {
  check_p0_overloaded_funtion1(data_p0::testcases1);
  check_p0_overloaded_funtion2(data_p0::testcases1);
}

TEST_CASE("Compute complex number defined from inner edge: case 2",
          "[phase_y][case2][p0]") {
  check_p0_overloaded_funtion1(data_p0::testcases2);
  check_p0_overloaded_funtion2(data_p0::testcases2);
}

namespace data_phase {

template <typename params>
struct testcase_t {
  static constexpr double yd = params::yd;
  static constexpr double yin = params::yin;
  static constexpr double q1 = params::q1;
  static constexpr double q2 = params::q2;
  static constexpr double zeta = params::zeta;
  static constexpr int dir = params::dir;
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
  static constexpr int dir = 1;
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
  static constexpr int dir = -1;
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
  static constexpr int dir = 1;
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
  static constexpr int dir = -1;
};
testcase_t<params4> testcases4[] = {
    {0, 1, 1},  {0.5, 0.5, 0},      {1, 0, 0},    {0.5, -0.5, 0},
    {0, -1, 0}, {-0.5, -0.5, 0.25}, {-1, 0, 0.5}, {-0.5, 0.5, 0.75}};

}  // namespace data_phase

void check_phase(const double phase, const double expected_phase) {
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
void check_phase_overloaded_function1(const testcase_t& testcases) {
  SECTION("overloaded function 1") {
    for (const auto& tc : testcases) {
      auto phase = computePhase(tc.yz, tc.vy, tc.yd, tc.yin, tc.q1, tc.q2,
                                tc.zeta, tc.dir);
      CAPTURE(tc.yz);
      CAPTURE(tc.vy);
      check_phase(phase, tc.expected_phase);
    }
  }
}

template <typename testcase_t>
void check_phase_overloaded_function2(const testcase_t& testcases) {
  SECTION("overloaded function 2") {
    for (const auto& tc : testcases) {
      Vec3D p_z = {0, tc.yz, 0};
      Vec3D v = {0, tc.vy, 0};
      Vec3D pd = {0, tc.yd, 0};
      Vec3D p_in = {0, tc.yin, 0};
      auto phase =
          computePhase(p_z, v, pd, p_in, tc.q1, tc.q2, tc.zeta, tc.dir);
      check_phase(phase, tc.expected_phase);
    }
  }
}

template <typename testcase_t>
void check_phase_overloaded_function3(const testcase_t& testcases) {
  SECTION("overloaded function 3") {
    for (const auto& tc : testcases) {
      auto pz = computeComplexZmp(tc.yz, tc.vy, tc.yd, tc.q1, tc.q2, tc.zeta);
      auto p0 = computeComplexInnerEdge(tc.yin, tc.yd, pz, tc.dir);
      auto phase = computePhase(pz, p0);
      check_phase(phase, tc.expected_phase);
    }
  }
}

TEST_CASE("Compute phase along y-axis: case 1", "[phase_y][phase][case1]") {
  check_phase_overloaded_function1(data_phase::testcases1);
  check_phase_overloaded_function2(data_phase::testcases1);
  check_phase_overloaded_function3(data_phase::testcases1);
}

TEST_CASE("Compute phase along y-axis: case 2", "[phase_y][phase][case2]") {
  check_phase_overloaded_function1(data_phase::testcases2);
  check_phase_overloaded_function2(data_phase::testcases2);
  check_phase_overloaded_function3(data_phase::testcases2);
}

TEST_CASE("Compute phase along y-axis: case 3", "[phase_y][phase][case3]") {
  check_phase_overloaded_function1(data_phase::testcases3);
  check_phase_overloaded_function2(data_phase::testcases3);
  check_phase_overloaded_function3(data_phase::testcases3);
}

TEST_CASE("Compute phase along y-axis: case 4", "[phase_y][phase][case4]") {
  check_phase_overloaded_function1(data_phase::testcases4);
  check_phase_overloaded_function2(data_phase::testcases4);
  check_phase_overloaded_function3(data_phase::testcases4);
}

namespace data_period {

struct testcase_t {
  double q1, q2, zeta;
  double expected_period;
};
testcase_t testcases1[] = {{1, 1, 1, zPIx2},
                           {0.5, 1, 1, zPIx2 / sqrt(0.5)},
                           {2, 1, 0.5, zPIx2 / (0.5 * sqrt(2))},
                           {0.1, 0.5, 0.1, zPIx2 / (0.1 * sqrt(0.05))}};
testcase_t testcases2[] = {{0, 1, 1, 0},  {1, 0, 1, 0},  {1, 1, 0, 0},
                           {-1, 1, 1, 0}, {1, -1, 1, 0}, {1, 1, -1, 0}};

}  // namespace data_period

TEST_CASE("Compute period of oscillation", "[phase_y][period]") {
  for (const auto& tc : data_period::testcases1) {
    auto period = computePeriod(tc.q1, tc.q2, tc.zeta);
    CHECK(period == Approx(tc.expected_period));
  }
}

TEST_CASE("The case where frequency equals to zero should be handled",
          "[phase_y][period][exception]") {
  zEchoOff();
  for (const auto& tc : data_period::testcases2) {
    auto period = computePeriod(tc.q1, tc.q2, tc.zeta);
    CHECK(period == Approx(tc.expected_period));
  }
  zEchoOn();
}

}  // namespace
}  // namespace phase_y
}  // namespace holon
