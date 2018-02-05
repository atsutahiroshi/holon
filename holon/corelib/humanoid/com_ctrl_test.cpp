/* com_ctrl - COM Controller
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

#include "holon/corelib/humanoid/com_ctrl.hpp"

#include <roki/rk_g.h>

#include "catch.hpp"
#include "holon/test/util/catch/custom_matchers.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

const double G = RK_G;

TEST_CASE("COM controller has the poles", "[corelib][humanoid]") {
  ComCtrl ctrl;
  Fuzzer fuzz;

  SECTION("default values of the poles should be 1 and 1") {
    CHECK(ctrl.q1() == 1.0);
    CHECK(ctrl.q2() == 1.0);
  }

  SECTION("set the values of the poles") {
    for (auto i = 0; i < 3; ++i) {
      double q1, q2;
      q1 = fuzz.get();
      q2 = fuzz.get();
      ctrl.set_q1(q1);
      ctrl.set_q2(q2);
      INFO("times: " << i);
      CHECK(ctrl.q1() == q1);
      CHECK(ctrl.q2() == q2);
    }
  }
}

TEST_CASE("compute desired zeta", "[corelib][humanoid]") {
  ComCtrl ctrl;

  SECTION("zeta should be computed according to the COM height") {
    struct testcase_t {
      double com_height;
      double expected_zeta_squared;
      double expected_zeta;
    } testcases[] = {{1, G, sqrt(G)},
                     {G, 1.0, 1.0},
                     {2, G / 2, sqrt(G / 2)},
                     {4, G / 4, sqrt(G / 4)}};

    for (auto c : testcases) {
      zVec3D pg = {0, 0, c.com_height};
      CHECK(ctrl.ComputeDesiredZetaSqr(&pg) == Approx(c.expected_zeta_squared));
      CHECK(ctrl.ComputeDesiredZeta(&pg) == Approx(c.expected_zeta));
    }
  }
  SECTION("return 0 when the given COM height was 0") {
    // Returning 0 when the height is 0 is due to avoiding zero-division,
    // but this is not theoritically correct.
    // This case should be handled as an exception somehow.
    // TODO(*): handle zero-division error correctly
    zVec3D pg = {0, 0, 0};
    zEchoOff();
    CHECK_FALSE(zIsInf(ctrl.ComputeDesiredZetaSqr(&pg)));
    CHECK(ctrl.ComputeDesiredZetaSqr(&pg) == 0.0);
    CHECK_FALSE(zIsInf(ctrl.ComputeDesiredZeta(&pg)));
    CHECK(ctrl.ComputeDesiredZeta(&pg) == 0.0);
    zEchoOn();
  }
  SECTION("return 0 when the given COM height was negative") {
    // Return 0 when a negative valued was given as the height of COM.
    // This should be handled as an exception as well.
    // TODO(*): handle the case where a negative value is given
    zVec3D pg = {0, 0, -1};
    zEchoOff();
    CHECK(ctrl.ComputeDesiredZetaSqr(&pg) == 0.0);
    CHECK_FALSE(zIsNan(ctrl.ComputeDesiredZeta(&pg)));
    CHECK(ctrl.ComputeDesiredZeta(&pg) == 0.0);
    zEchoOn();
  }
}

TEST_CASE("compute desired ZMP position from the referential COM position",
          "[corelib][humanoid]") {
  ComCtrl ctrl;

  SECTION("referential COM position is set as (0, 0, 1)") {
    zVec3D ref_com_pos = {0, 0, 1};
    zVec3D com_pos = {0, 0, 1};
    zVec3D com_vel = {0, 0, 0};
    zVec3D desired_zmp_pos;
    zVec3D expected_zmp_pos = {0, 0, 0};

    ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                   &desired_zmp_pos);
    CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
  }
}

}  // namespace
}  // namespace holon
