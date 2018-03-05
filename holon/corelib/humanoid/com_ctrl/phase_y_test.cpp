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

TEST_CASE("Compute frequency of oscillation", "[phase_y]") {
  for (const auto& tc : data_omega::testcases1) {
    auto omega = computeFrequency(tc.q1, tc.q2, tc.zeta);
    CHECK(omega == Approx(tc.expected_omega));
  }
}

TEST_CASE("Frequency of oscillation must be positive", "[phase_y][exception]") {
  zEchoOff();
  for (const auto& tc : data_omega::testcases2) {
    auto omega = computeFrequency(tc.q1, tc.q2, tc.zeta);
    CHECK(omega == Approx(tc.expected_omega));
  }
  zEchoOn();
}

}  // namespace
}  // namespace phase_y
}  // namespace holon
