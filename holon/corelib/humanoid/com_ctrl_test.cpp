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
  Fuzzer fuzz;

  SECTION("use constructor with no paramters") {
    ComCtrl ctrl;

    SECTION("default values of the poles should be 1 and 1") {
      CHECK(ctrl.q1() == 1.0);
      CHECK(ctrl.q2() == 1.0);
    }

    SECTION("set the values of the poles") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.set_q1(q1);
        ctrl.set_q2(q2);
        INFO("times: " << i);
        CHECK(ctrl.q1() == q1);
        CHECK(ctrl.q2() == q2);
      }
    }

    SECTION("set the values of the poles with chaining methods") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.set_q1(q1).set_q2(q2);
        INFO("times: " << i);
        CHECK(ctrl.q1() == q1);
        CHECK(ctrl.q2() == q2);
      }
    }
  }

  SECTION("use constructor with paramters") {
    for (auto i = 0; i < 3; ++i) {
      double q1 = fuzz.get();
      double q2 = fuzz.get();
      ComCtrl ctrl(q1, q2);
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

TEST_CASE("compute desired ZMP position to regulate COM at (0, 0, G)",
          "[corelib][humanoid]") {
  ComCtrl ctrl;
  zVec3D ref_com_pos = {0, 0, G};

  SECTION("case: q1 = 1, q2 = 1, zeta = 1") {
    // fixed paramters
    REQUIRE(ctrl.model().ComputeZeta(&ref_com_pos) == Approx(1.0));
    REQUIRE(ctrl.q1() == 1.0);
    REQUIRE(ctrl.q2() == 1.0);

    SECTION("check only along x-axis") {
      struct testcase_t {
        double com_pos_x;
        double com_vel_x;
        double expected_zmp_pos_x;
      } testcases[] = {
          {0, 0, 0}, {1, 0, 2}, {3, -1, 4}, {0, -2, -4}, {-2, 2, 0}};

      for (auto& c : testcases) {
        zVec3D com_pos = {c.com_pos_x, 0, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {c.com_vel_x, 0, 0};
        zVec3D expected_zmp_pos = {c.expected_zmp_pos_x, 0, 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
    SECTION("check only along y-axis") {
      struct testcase_t {
        double com_pos_y;
        double com_vel_y;
        double expected_zmp_pos_y;
      } testcases[] = {
          {0, 0, 0}, {1, 0, 2}, {3, -1, 4}, {0, -2, -4}, {-2, 2, 0}};

      for (auto& c : testcases) {
        zVec3D com_pos = {0, c.com_pos_y, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {0, c.com_vel_y, 0};
        zVec3D expected_zmp_pos = {0, c.expected_zmp_pos_y, 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
  }

  SECTION("case: q1 = 1, q2 = 0.5, zeta = 1") {
    // fixed paramters
    REQUIRE(ctrl.model().ComputeZeta(&ref_com_pos) == Approx(1.0));
    ctrl.set_q2(0.5);
    REQUIRE(ctrl.q1() == 1.0);
    REQUIRE(ctrl.q2() == 0.5);

    SECTION("check only along x-axis") {
      struct testcase_t {
        double com_pos_x;
        double com_vel_x;
        double expected_zmp_pos_x;
      } testcases[] = {
          {0, 0, 0}, {1, 0, 1.5}, {3, -1, 3}, {0, -2, -3}, {-2, 2, 0}};

      for (auto& c : testcases) {
        zVec3D com_pos = {c.com_pos_x, 0, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {c.com_vel_x, 0, 0};
        zVec3D expected_zmp_pos = {c.expected_zmp_pos_x, 0, 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
    SECTION("check only along y-axis") {
      struct testcase_t {
        double com_pos_y;
        double com_vel_y;
        double expected_zmp_pos_y;
      } testcases[] = {
          {0, 0, 0}, {1, 0, 1.5}, {3, -1, 3}, {0, -2, -3}, {-2, 2, 0}};

      for (auto& c : testcases) {
        zVec3D com_pos = {0, c.com_pos_y, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {0, c.com_vel_y, 0};
        zVec3D expected_zmp_pos = {0, c.expected_zmp_pos_y, 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
  }
  SECTION("case: q1 = 1.2, q2 = 0.8, zeta = 1") {
    // fixed paramters
    REQUIRE(ctrl.model().ComputeZeta(&ref_com_pos) == Approx(1.0));
    ctrl.set_q1(1.2);
    ctrl.set_q2(0.8);
    REQUIRE(ctrl.q1() == 1.2);
    REQUIRE(ctrl.q2() == 0.8);

    SECTION("check only along x-axis") {
      struct testcase_t {
        double com_pos_x;
        double com_vel_x;
        double expected_zmp_pos_x;
      } testcases[] = {
          {0, 0, 0}, {1, 0, 1.96}, {3, -1, 3.88}, {0, -2, -4}, {-2, 2, 0.08}};

      for (auto& c : testcases) {
        zVec3D com_pos = {c.com_pos_x, 0, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {c.com_vel_x, 0, 0};
        zVec3D expected_zmp_pos = {c.expected_zmp_pos_x, 0, 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
    SECTION("check only along y-axis") {
      struct testcase_t {
        double com_pos_y;
        double com_vel_y;
        double expected_zmp_pos_y;
      } testcases[] = {
          {0, 0, 0}, {1, 0, 1.96}, {3, -1, 3.88}, {0, -2, -4}, {-2, 2, 0.08}};

      for (auto& c : testcases) {
        zVec3D com_pos = {0, c.com_pos_y, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {0, c.com_vel_y, 0};
        zVec3D expected_zmp_pos = {0, c.expected_zmp_pos_y, 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
  }
}

TEST_CASE("compute desired ZMP position to regulate COM when q1 = 1, q2 = 1",
          "[corelib][humanoid]") {
  ComCtrl ctrl;
  REQUIRE(ctrl.q1() == 1.0);
  REQUIRE(ctrl.q2() == 1.0);

  SECTION("case: referential COM position = (1, 0.5, G)") {
    zVec3D ref_com_pos = {1, 0.5, G};
    REQUIRE(ctrl.model().ComputeZeta(&ref_com_pos) == Approx(1.0));

    SECTION("check only along x-axis") {
      struct testcase_t {
        double com_pos_x;
        double com_vel_x;
        double expected_zmp_pos_x;
      } testcases[] = {
          {0, 0, -1}, {1, 0, 1}, {3, -1, 3}, {0, -2, -5}, {-2, 3, 1}};

      for (auto& c : testcases) {
        zVec3D com_pos = {c.com_pos_x, 0, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {c.com_vel_x, 0, 0};
        zVec3D expected_zmp_pos = {c.expected_zmp_pos_x,
                                   -zVec3DElem(&ref_com_pos, zY), 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
    SECTION("check only along y-axis") {
      struct testcase_t {
        double com_pos_y;
        double com_vel_y;
        double expected_zmp_pos_y;
      } testcases[] = {
          {0, 0, -0.5}, {1, 0, 1.5}, {3, -1, 3.5}, {0, -2, -4.5}, {-2, 3, 1.5}};

      for (auto& c : testcases) {
        zVec3D com_pos = {0, c.com_pos_y, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {0, c.com_vel_y, 0};
        zVec3D expected_zmp_pos = {-zVec3DElem(&ref_com_pos, zX),
                                   c.expected_zmp_pos_y, 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
  }
}

TEST_CASE("compute desired ZMP position to regulate COM when q1 = 1, q2 = 1.5",
          "[corelib][humanoid]") {
  ComCtrl ctrl;
  ctrl.set_q2(1.5);
  REQUIRE(ctrl.q1() == 1.0);
  REQUIRE(ctrl.q2() == 1.5);

  SECTION("case: referential COM position = (1, -1, G)") {
    zVec3D ref_com_pos = {1, -1, G};
    REQUIRE(ctrl.model().ComputeZeta(&ref_com_pos) == Approx(1.0));

    SECTION("check only along x-axis") {
      struct testcase_t {
        double com_pos_x;
        double com_vel_x;
        double expected_zmp_pos_x;
      } testcases[] = {
          {0, 0, -1.5}, {1, 0, 1}, {3, -1, 3.5}, {0, -2, -6.5}, {-2, 3, 1}};

      for (auto& c : testcases) {
        zVec3D com_pos = {c.com_pos_x, 0, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {c.com_vel_x, 0, 0};
        zVec3D expected_zmp_pos = {c.expected_zmp_pos_x,
                                   -1.5 * zVec3DElem(&ref_com_pos, zY), 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
    SECTION("check only along y-axis") {
      struct testcase_t {
        double com_pos_y;
        double com_vel_y;
        double expected_zmp_pos_y;
      } testcases[] = {
          {0, 0, 1.5}, {1, 0, 4}, {3, -1, 6.5}, {0, -2, -3.5}, {-2, 3, 4}};

      for (auto& c : testcases) {
        zVec3D com_pos = {0, c.com_pos_y, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {0, c.com_vel_y, 0};
        zVec3D expected_zmp_pos = {-1.5 * zVec3DElem(&ref_com_pos, zX),
                                   c.expected_zmp_pos_y, 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
  }
}

TEST_CASE("compute desired ZMP position to regulate COM with q1 = 1, q2 = 1.5",
          "[corelib][humanoid]") {
  ComCtrl ctrl;
  ctrl.set_q2(1.5);
  REQUIRE(ctrl.q1() == 1.0);
  REQUIRE(ctrl.q2() == 1.5);

  SECTION("case: referential COM position = (0, 0.5, 0.5*G)") {
    zVec3D ref_com_pos = {0, 0.5, 0.5 * G};
    REQUIRE(ctrl.model().ComputeZeta(&ref_com_pos) == Approx(sqrt(2)));

    SECTION("check only along x-axis") {
      struct testcase_t {
        double com_pos_x;
        double com_vel_x;
        double expected_zmp_pos_x;
      } testcases[] = {{0, 0, 0},
                       {1, 0, 2.5},
                       {3, -2, 2.5 * (3 - sqrt(2))},
                       {0, -1, -1.25 * sqrt(2)},
                       {-2, 3, 2.5 * (-2 + 1.5 * sqrt(2))}};

      for (auto& c : testcases) {
        zVec3D com_pos = {c.com_pos_x, 0, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {c.com_vel_x, 0, 0};
        zVec3D expected_zmp_pos = {c.expected_zmp_pos_x,
                                   -1.5 * zVec3DElem(&ref_com_pos, zY), 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
    SECTION("check only along y-axis") {
      struct testcase_t {
        double com_pos_y;
        double com_vel_y;
        double expected_zmp_pos_y;
      } testcases[] = {{0, 0, -0.75},
                       {1, 0, 1.75},
                       {3, -2, 2.5 * (3 - sqrt(2)) - 0.75},
                       {0, -1, -1.25 * sqrt(2) - 0.75},
                       {-2, 3, 2.5 * (-2 + 1.5 * sqrt(2)) - 0.75}};

      for (auto& c : testcases) {
        zVec3D com_pos = {0, c.com_pos_y, zVec3DElem(&ref_com_pos, zZ)};
        zVec3D com_vel = {0, c.com_vel_y, 0};
        zVec3D expected_zmp_pos = {zVec3DElem(&ref_com_pos, zX),
                                   c.expected_zmp_pos_y, 0};
        zVec3D desired_zmp_pos;
        ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos, Catch::Equals(&expected_zmp_pos));
      }
    }
  }
}

}  // namespace
}  // namespace holon
