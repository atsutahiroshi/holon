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

TEST_CASE("check if control poles on each axis can be assigned",
          "[corelib][humanoid]") {
  Fuzzer fuzz;

  SECTION("control poles along x-axis") {
    ComCtrl ctrl;

    SECTION("set the values of the poles") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.x().set_q1(q1);
        ctrl.x().set_q2(q2);
        CHECK(ctrl.x().q1() == q1);
        CHECK(ctrl.x().q2() == q2);
      }
    }

    SECTION("set the values of the poles with chaining methods") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.x().set_q1(q1).set_q2(q2);
        CHECK(ctrl.x().q1() == q1);
        CHECK(ctrl.x().q2() == q2);
      }
    }
  }

  SECTION("control poles along y-axis") {
    ComCtrl ctrl;

    SECTION("set the values of the poles") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.y().set_q1(q1);
        ctrl.y().set_q2(q2);
        CHECK(ctrl.y().q1() == q1);
        CHECK(ctrl.y().q2() == q2);
      }
    }

    SECTION("set the values of the poles with chaining methods") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.y().set_q1(q1).set_q2(q2);
        CHECK(ctrl.y().q1() == q1);
        CHECK(ctrl.y().q2() == q2);
      }
    }
  }
}

TEST_CASE("com_ctrl: testing of initialization", "[corelib][humanoid]") {
  ComCtrl ctrl;

  SECTION("commanded COM position") {
    zVec3D expected_cmd_com_pos = {0, 0, 1};
    CHECK_THAT(ctrl.cmd_com_position(),
               Catch::Matchers::Equals(&expected_cmd_com_pos));
  }
  SECTION("desired ZMP position") {
    zVec3D expected_des_zmp_pos = {0, 0, 0};
    CHECK_THAT(ctrl.des_zmp_position(),
               Catch::Matchers::Equals(&expected_des_zmp_pos));
  }
  SECTION("desired value of zeta") { CHECK(ctrl.des_zeta() == sqrt(G)); }
}

TEST_CASE("com_ctrl: testing of accessors/mutators", "[corelib][humanoid]") {
  ComCtrl ctrl;
  Fuzzer fuzz;

  SECTION("commanded COM position") {
    zVec3D new_cmd_com_pos;
    fuzz.randomize(&new_cmd_com_pos);
    ctrl.set_cmd_com_position(&new_cmd_com_pos);
    REQUIRE_THAT(ctrl.cmd_com_position(),
                 Catch::Matchers::Equals(&new_cmd_com_pos));
  }

  SECTION("time step") {
    Fuzzer fuzz(0.0001, 0.1);
    double dt = fuzz.get();
    ctrl.set_time_step(dt);
    CHECK(ctrl.time_step() == dt);
    CHECK(ctrl.model().time_step() == dt);
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
      CHECK(ctrl.computeDesiredZetaSqr(&pg) == Approx(c.expected_zeta_squared));
      CHECK(ctrl.computeDesiredZeta(&pg) == Approx(c.expected_zeta));
    }
  }
  SECTION("return 0 when the given COM height was 0") {
    // Returning 0 when the height is 0 is due to avoiding zero-division,
    // but this is not theoritically correct.
    // This case should be handled as an exception somehow.
    // TODO(*): handle zero-division error correctly
    zVec3D pg = {0, 0, 0};
    zEchoOff();
    CHECK_FALSE(zIsInf(ctrl.computeDesiredZetaSqr(&pg)));
    CHECK(ctrl.computeDesiredZetaSqr(&pg) == 0.0);
    CHECK_FALSE(zIsInf(ctrl.computeDesiredZeta(&pg)));
    CHECK(ctrl.computeDesiredZeta(&pg) == 0.0);
    zEchoOn();
  }
  SECTION("return 0 when the given COM height was negative") {
    // Return 0 when a negative valued was given as the height of COM.
    // This should be handled as an exception as well.
    // TODO(*): handle the case where a negative value is given
    zVec3D pg = {0, 0, -1};
    zEchoOff();
    CHECK(ctrl.computeDesiredZetaSqr(&pg) == 0.0);
    CHECK_FALSE(zIsNan(ctrl.computeDesiredZeta(&pg)));
    CHECK(ctrl.computeDesiredZeta(&pg) == 0.0);
    zEchoOn();
  }
}

SCENARIO("compute desired ZMP position", "[corelib][humanoid]") {
  GIVEN("qx1 = 1.0, qx2 = 1.0, qy1 = 1.0, qy2 = 1.0, ref_com_pos = (0, 0, G)") {
    ComCtrl ctrl;
    zVec3D ref_com_pos = {0, 0, G};
    double desired_zeta = ctrl.computeDesiredZeta(&ref_com_pos);

    REQUIRE(desired_zeta == Approx(1.0));
    REQUIRE(ctrl.x().q1() == 1);
    REQUIRE(ctrl.x().q2() == 1);
    REQUIRE(ctrl.y().q1() == 1);
    REQUIRE(ctrl.y().q2() == 1);

    WHEN("com_pos = (0, 0, G), com_vel = (0, 0, 0)") {
      zVec3D com_pos = {0, 0, G};
      zVec3D com_vel = {0, 0, 0};

      THEN("desire_zmp_pos = (0, 0, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {0, 0, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
    WHEN("com_pos = (1, 3, G), com_vel = (0, -1, 0)") {
      zVec3D com_pos = {1, 3, G};
      zVec3D com_vel = {0, -1, 0};

      THEN("desire_zmp_pos = (2, 4, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {2, 4, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
    WHEN("com_pos = (0, -2, G), com_vel = (-2, 2, 0)") {
      zVec3D com_pos = {0, -2, G};
      zVec3D com_vel = {-2, 2, 0};

      THEN("desire_zmp_pos = (-4, 0, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {-4, 0, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
  }

  GIVEN("qx1 = 1, qx2 = 0.5, qy1 = 1.2, qy2 = 0.8, ref_com_pos = (0, 0, G)") {
    ComCtrl ctrl;
    zVec3D ref_com_pos = {0, 0, G};
    double desired_zeta = ctrl.computeDesiredZeta(&ref_com_pos);

    ctrl.x().set_q1(1.0).set_q2(0.5);
    ctrl.y().set_q1(1.2).set_q2(0.8);

    REQUIRE(desired_zeta == Approx(1.0));
    REQUIRE(ctrl.x().q1() == 1.0);
    REQUIRE(ctrl.x().q2() == 0.5);
    REQUIRE(ctrl.y().q1() == 1.2);
    REQUIRE(ctrl.y().q2() == 0.8);

    WHEN("com_pos = (1, 1, G), com_vel = (0, 0, 0)") {
      zVec3D com_pos = {1, 1, G};
      zVec3D com_vel = {0, 0, 0};

      THEN("desire_zmp_pos = (1.5, 1.96, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {1.5, 1.96, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
    WHEN("com_pos = (-2, 3, G), com_vel = (2, -1, 0)") {
      zVec3D com_pos = {-2, 3, G};
      zVec3D com_vel = {2, -1, 0};

      THEN("desire_zmp_pos = (0, 3.88, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {0, 3.88, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
  }
  GIVEN(
      "qx1 = 1.0, qx2 = 1.5, qy1 = 1.0, qy2 = 1.0, ref_com_pos = (1, 0.5, G)") {
    ComCtrl ctrl;
    zVec3D ref_com_pos = {1, 0.5, G};
    double desired_zeta = ctrl.computeDesiredZeta(&ref_com_pos);

    ctrl.x().set_q2(1.5);

    REQUIRE(desired_zeta == Approx(1.0));
    REQUIRE(ctrl.x().q1() == 1);
    REQUIRE(ctrl.x().q2() == 1.5);
    REQUIRE(ctrl.y().q1() == 1);
    REQUIRE(ctrl.y().q2() == 1);

    WHEN("com_pos = (1, 0, G), com_vel = (0, 0, 0)") {
      zVec3D com_pos = {1, 0, G};
      zVec3D com_vel = {0, 0, 0};

      THEN("desire_zmp_pos = (1, -0.5, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {1, -0.5, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
    WHEN("com_pos = (0, 3, G), com_vel = (-2, -1, 0)") {
      zVec3D com_pos = {0, 3, G};
      zVec3D com_vel = {-2, -1, 0};

      THEN("desire_zmp_pos = (-6.5, 3.5, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {-6.5, 3.5, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
    WHEN("com_pos = (-2, 0, G), com_vel = (3, -2, 0)") {
      zVec3D com_pos = {-2, 0, G};
      zVec3D com_vel = {3, -2, 0};

      THEN("desire_zmp_pos = (1, -4.5, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {1, -4.5, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
  }

  GIVEN(
      "qx1 = 1.0, qx2 = 1.5, qy1 = 1.0, qy2 = 1.5, "
      "ref_com_pos = (0, 0.5, 0.5*G)") {
    ComCtrl ctrl;
    zVec3D ref_com_pos = {0, 0.5, 0.5 * G};
    double desired_zeta = ctrl.computeDesiredZeta(&ref_com_pos);

    ctrl.x().set_q2(1.5);
    ctrl.y().set_q2(1.5);

    REQUIRE(desired_zeta == Approx(sqrt(2.0)));
    REQUIRE(ctrl.x().q1() == 1);
    REQUIRE(ctrl.x().q2() == 1.5);
    REQUIRE(ctrl.y().q1() == 1);
    REQUIRE(ctrl.y().q2() == 1.5);

    WHEN("com_pos = (0, 0, G), com_vel = (0, 0, 0)") {
      zVec3D com_pos = {0, 0, G};
      zVec3D com_vel = {0, 0, 0};

      THEN("desire_zmp_pos = (0, -0.75, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {0, -0.75, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
    WHEN("com_pos = (1, 3, G), com_vel = (0, -2, 0)") {
      zVec3D com_pos = {1, 3, G};
      zVec3D com_vel = {0, -2, 0};

      THEN("desire_zmp_pos = (2.5, 2.5*(3-sqrt(2))-0.75, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {2.5, 2.5 * (3 - sqrt(2)) - 0.75, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
    WHEN("com_pos = (-2, 0, G), com_vel = (3, -1, 0)") {
      zVec3D com_pos = {-2, 0, G};
      zVec3D com_vel = {3, -1, 0};

      THEN("desire_zmp_pos = (2.5*(-2+1.5*sqrt(2)), -1.25*sqrt(2)-0.75, 0)") {
        zVec3D desired_zmp_pos;
        zVec3D expected_zmp_pos = {2.5 * (-2 + 1.5 * sqrt(2)),
                                   -1.25 * sqrt(2) - 0.75, 0};
        ctrl.computeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                       desired_zeta, &desired_zmp_pos);
        CHECK_THAT(&desired_zmp_pos,
                   Catch::Matchers::Equals(&expected_zmp_pos));
      }
    }
  }
}

TEST_CASE("check if time step is modified after update",
          "[corelib][humanoid]") {
  ComCtrl ctrl;
  Fuzzer fuzz(0.0001, 0.1);
  double dt1 = fuzz.get();
  double dt2 = fuzz.get();

  REQUIRE(ctrl.time_step() != dt1);
  ctrl.update(dt1);
  CHECK(ctrl.time_step() == dt1);
  CHECK(ctrl.model().time_step() == dt1);
  ctrl.update();
  CHECK(ctrl.time_step() == dt1);
  CHECK(ctrl.model().time_step() == dt1);

  REQUIRE(ctrl.time_step() != dt2);
  ctrl.update(dt2);
  CHECK(ctrl.time_step() == dt2);
  CHECK(ctrl.model().time_step() == dt2);
  ctrl.update();
  CHECK(ctrl.time_step() == dt2);
  CHECK(ctrl.model().time_step() == dt2);
}

TEST_CASE("check if desired ZMP position is modified after update",
          "[corelib][humanoid]") {
  ComCtrl ctrl;

  struct testcase_t {
    zVec3D cmd_com_pos;
  } testcases[] = {{{0, 0, 1}}, {{0.1, 0, 1}}, {{0.1, -0.1, 1}}};
  for (auto& c : testcases) {
    zVec3D expected_des_zmp_pos;
    double expected_des_zeta;

    expected_des_zeta = ctrl.computeDesiredZeta(&c.cmd_com_pos);
    ctrl.computeDesiredZmpPosition(&c.cmd_com_pos, ctrl.model().com_position(),
                                   ctrl.model().com_velocity(),
                                   expected_des_zeta, &expected_des_zmp_pos);

    ctrl.set_cmd_com_position(&c.cmd_com_pos);
    ctrl.update();
    CHECK(ctrl.des_zeta() == expected_des_zeta);
    CHECK_THAT(ctrl.des_zmp_position(),
               Catch::Matchers::Equals(&expected_des_zmp_pos));
  }
}

SCENARIO("controller can regulate COM position at a point",
         "[corelib][humanoid]") {
  GIVEN("command that COM position be at (0.1, -0.1, 1)") {
    ComCtrl ctrl;
    zVec3D cmd_com_pos = {0.1, -0.1, 1};
    ctrl.set_cmd_com_position(&cmd_com_pos);

    WHEN("at first") {
      THEN("COM position is at (0, 0, 1)") {
        zVec3D expected_com_pos = {0, 0, 1};
        CHECK_THAT(const_cast<zVec3D*>(ctrl.model().com_position()),
                   Catch::Matchers::Equals(&expected_com_pos));
      }
    }
    WHEN("update until 0.1 sec") {
      double t = 0;
      while (t < 0.1) {
        ctrl.update();
        t += ctrl.time_step();
      }
      THEN("COM position is between (0, 0, 1) and (0.1, -0.1, 1)") {
        CAPTURE(ctrl.model().com_position());
        CHECK(zVec3DElem(ctrl.model().com_position(), zX) > 0.0);
        CHECK(zVec3DElem(ctrl.model().com_position(), zX) < 0.1);
        CHECK(zVec3DElem(ctrl.model().com_position(), zY) < 0.0);
        CHECK(zVec3DElem(ctrl.model().com_position(), zY) > -0.1);
      }
    }
    WHEN("update until 10 sec") {
      double t = 0;
      while (t < 10) {
        ctrl.update();
        t += ctrl.time_step();
      }
      THEN("COM lies at (0.1, -0.1, 1)") {
        CHECK_THAT(const_cast<zVec3D*>(ctrl.model().com_position()),
                   Catch::Matchers::Equals(&cmd_com_pos));
      }
    }
  }
}

}  // namespace
}  // namespace holon
