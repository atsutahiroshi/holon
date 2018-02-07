/* com_ctrl_y - COM controller along y axis
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

#include "holon/corelib/humanoid/com_ctrl_y.hpp"

#include <cure/cure_misc.h>

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

TEST_CASE("control poles along y-axis can be assigned", "[corelib][humanoid]") {
  Fuzzer fuzz;

  SECTION("instantiate with no paramters") {
    ComCtrlY ctrl;

    SECTION("default values of q1 and q2 are 1") {
      CHECK(ctrl.q1() == 1.0);
      CHECK(ctrl.q2() == 1.0);
    }

    SECTION("q1, q2 can be assigned") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.set_q1(q1);
        ctrl.set_q2(q2);
        CHECK(ctrl.q1() == q1);
        CHECK(ctrl.q2() == q2);
      }
    }

    SECTION("q1, q2 can be assigned with chaining method") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.set_q1(q1).set_q2(q2);
        CHECK(ctrl.q1() == q1);
        CHECK(ctrl.q2() == q2);
      }
    }
  }

  SECTION("instantiate with paramters") {
    for (auto i = 0; i < 3; ++i) {
      double q1 = fuzz.get();
      double q2 = fuzz.get();

      ComCtrlY ctrl(q1, q2);
      CHECK(ctrl.q1() == q1);
      CHECK(ctrl.q2() == q2);
    }
  }

  SECTION("instantiate with paramters then set arbitrary value") {
    for (auto i = 0; i < 3; ++i) {
      double q1 = fuzz.get();
      double q2 = fuzz.get();

      ComCtrlY ctrl(q1, q2);
      CHECK(ctrl.q1() == q1);
      CHECK(ctrl.q2() == q2);

      q1 = fuzz.get();
      q2 = fuzz.get();
      ctrl.set_q1(q1);
      ctrl.set_q2(q2);
      CHECK(ctrl.q1() == q1);
      CHECK(ctrl.q2() == q2);
    }
  }
}

TEST_CASE("compute desired ZMP position along y-axis to regulate at 0",
          "[corelib][humanoid]") {
  ComCtrlY ctrl;

  SECTION("case: yd = 0, q1 = 1, q2 = 1, zeta = 1") {
    double zeta = 1;
    double yd = 0;
    REQUIRE(ctrl.q1() == 1);
    REQUIRE(ctrl.q2() == 1);

    struct testcase_t {
      double y, vy;
      double expected_yz;
    } testcases[] = {{0, 0, 0}, {1, 0, 2}, {3, -1, 4}, {0, -2, -4}, {-2, 2, 0}};

    for (auto& c : testcases) {
      CHECK(ctrl.computeDesiredZmpPosition(yd, c.y, c.vy, zeta) ==
            Approx(c.expected_yz));
    }
  }

  SECTION("case: yd = 0, q1 = 1, q2 = 0.5, zeta = 1") {
    double zeta = 1;
    double yd = 0;
    ctrl.set_q2(0.5);
    REQUIRE(ctrl.q1() == 1);
    REQUIRE(ctrl.q2() == 0.5);

    struct testcase_t {
      double y, vy;
      double expected_yz;
    } testcases[] = {
        {0, 0, 0}, {1, 0, 1.5}, {3, -1, 3}, {0, -2, -3}, {-2, 2, 0}};

    for (auto& c : testcases) {
      CHECK(ctrl.computeDesiredZmpPosition(yd, c.y, c.vy, zeta) ==
            Approx(c.expected_yz));
    }
  }

  SECTION("case: yd = 0, q1 = 1.2, q2 = 0.8, zeta = 1") {
    double zeta = 1;
    double yd = 0;
    ctrl.set_q1(1.2);
    ctrl.set_q2(0.8);
    REQUIRE(ctrl.q1() == 1.2);
    REQUIRE(ctrl.q2() == 0.8);

    struct testcase_t {
      double y, vy;
      double expected_yz;
    } testcases[] = {
        {0, 0, 0}, {1, 0, 1.96}, {3, -1, 3.88}, {0, -2, -4}, {-2, 2, 0.08}};

    for (auto& c : testcases) {
      CHECK(ctrl.computeDesiredZmpPosition(yd, c.y, c.vy, zeta) ==
            Approx(c.expected_yz));
    }
  }

  SECTION("case: yd = 0.5, q1 = 1, q2 = 1, zeta = 1") {
    double zeta = 1;
    double yd = 0.5;
    REQUIRE(ctrl.q1() == 1.0);
    REQUIRE(ctrl.q2() == 1.0);

    struct testcase_t {
      double y, vy;
      double expected_yz;
    } testcases[] = {
        {0, 0, -0.5}, {1, 0, 1.5}, {3, -1, 3.5}, {0, -2, -4.5}, {-2, 3, 1.5}};

    for (auto& c : testcases) {
      CHECK(ctrl.computeDesiredZmpPosition(yd, c.y, c.vy, zeta) ==
            Approx(c.expected_yz));
    }
  }

  SECTION("case: yd = 1, q1 = 1, q2 = 1.5, zeta = 1") {
    double zeta = 1;
    double yd = -1;
    ctrl.set_q2(1.5);
    REQUIRE(ctrl.q1() == 1.0);
    REQUIRE(ctrl.q2() == 1.5);

    struct testcase_t {
      double y, vy;
      double expected_yz;
    } testcases[] = {
        {0, 0, 1.5}, {1, 0, 4}, {3, -1, 6.5}, {0, -2, -3.5}, {-2, 3, 4}};

    for (auto& c : testcases) {
      CHECK(ctrl.computeDesiredZmpPosition(yd, c.y, c.vy, zeta) ==
            Approx(c.expected_yz));
    }
  }

  SECTION("case: yd = 0.5, q1 = 1, q2 = 1.5, zeta = sqrt(2)") {
    double zeta = sqrt(2);
    double yd = 0.5;
    ctrl.set_q2(1.5);
    REQUIRE(ctrl.q1() == 1.0);
    REQUIRE(ctrl.q2() == 1.5);

    struct testcase_t {
      double y, vy;
      double expected_yz;
    } testcases[] = {{0, 0, -0.75},
                     {1, 0, 1.75},
                     {3, -2, 2.5 * (3 - sqrt(2)) - 0.75},
                     {0, -1, -1.25 * sqrt(2) - 0.75},
                     {-2, 3, 2.5 * (-2 + 1.5 * sqrt(2)) - 0.75}};

    for (auto& c : testcases) {
      CHECK(ctrl.computeDesiredZmpPosition(yd, c.y, c.vy, zeta) ==
            Approx(c.expected_yz));
    }
  }
}

TEST_CASE("handle the case where zeta is non-positive on y-axis",
          "[corelib][humanoid]") {
  ComCtrlY ctrl;
  zEchoOff();
  REQUIRE(ctrl.computeDesiredZmpPosition(1, 0, 0, 0) == 0);
  REQUIRE(ctrl.computeDesiredZmpPosition(1, 0, 0, -1) == 0);
  zEchoOn();
}

}  // namespace
}  // namespace holon
