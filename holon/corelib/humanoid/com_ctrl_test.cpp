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

using Catch::Matchers::Equals;

const double G = RK_G;

TEST_CASE("ComCtrl: constructor", "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  SECTION("check if member pointers are preserved") {
    REQUIRE(&ctrl.states());
    REQUIRE(&ctrl.inputs());
    REQUIRE(&ctrl.outputs());
    REQUIRE(&ctrl.cmds());
    REQUIRE(ctrl.model().data().get() == &ctrl.states());
  }

  SECTION("check if initial COM position be initialized") {
    CHECK_THAT(ctrl.initial_com_position(), Equals(ctrl.states().com_position));
  }
}

TEST_CASE("ComCtrl::getUserCommands() provides a pointer to user commands",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;

  auto cmd = ctrl.getUserCommands();
  CHECK(cmd->com_position == nullopt);
  CHECK(cmd->com_position != kVec3DZero);
  CHECK_FALSE(cmd->com_position);
  CHECK_FALSE(cmd->com_position.has_value());
  CHECK_THROWS_AS(cmd->com_position.value(), bad_optional_access);
  CHECK(cmd->com_position.value_or(kVec3DZero) == kVec3DZero);

  cmd->com_position = Vec3D(0, 0, 0.42);
  CHECK(cmd->com_position == Vec3D(0, 0, 0.42));
  CHECK(cmd->com_position.value() == Vec3D(0, 0, 0.42));
  CHECK(cmd->com_position.value_or(kVec3DZero) == Vec3D(0, 0, 0.42));
}

TEST_CASE("ComCtrl::reset(const Vec3D&) should reset initial COM position") {
  ComCtrl ctrl;
  Vec3D supposed_pi = {0, 0, 1.5};
  REQUIRE_THAT(ctrl.initial_com_position(), !Equals(supposed_pi));

  ctrl.reset(supposed_pi);
  CHECK_THAT(ctrl.initial_com_position(), Equals(supposed_pi));
  CHECK_THAT(ctrl.states().com_position, Equals(supposed_pi));
  CHECK_THAT(ctrl.states().com_velocity, Equals(kVec3DZero));
}

SCENARIO("ComCtrl: compute desired ZMP position",
         "[corelib][humanoid][ComCtrl]") {
  GIVEN("qx1 = 1.0, qx2 = 1.0, qy1 = 1.0, qy2 = 1.0, ref_com_pos = (0, 0, G)") {
    ComCtrl ctrl;
    Vec3D ref_com_pos = {0, 0, G};
    double desired_zeta =
        ctrl.model().computeZeta(ref_com_pos, kVec3DZero, kVec3DZero);

    REQUIRE(desired_zeta == Approx(1.0));
    REQUIRE(ctrl.x().q1() == 1);
    REQUIRE(ctrl.x().q2() == 1);
    REQUIRE(ctrl.y().q1() == 1);
    REQUIRE(ctrl.y().q2() == 1);

    WHEN("com_pos = (0, 0, G), com_vel = (0, 0, 0)") {
      Vec3D com_pos = {0, 0, G};
      Vec3D com_vel = {0, 0, 0};

      THEN("desire_zmp_pos = (0, 0, 0)") {
        Vec3D expected_zmp_pos = {0, 0, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
    WHEN("com_pos = (1, 3, G), com_vel = (0, -1, 0)") {
      Vec3D com_pos = {1, 3, G};
      Vec3D com_vel = {0, -1, 0};

      THEN("desire_zmp_pos = (2, 4, 0)") {
        Vec3D expected_zmp_pos = {2, 4, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
    WHEN("com_pos = (0, -2, G), com_vel = (-2, 2, 0)") {
      Vec3D com_pos = {0, -2, G};
      Vec3D com_vel = {-2, 2, 0};

      THEN("desire_zmp_pos = (-4, 0, 0)") {
        Vec3D expected_zmp_pos = {-4, 0, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
  }

  GIVEN("qx1 = 1, qx2 = 0.5, qy1 = 1.2, qy2 = 0.8, ref_com_pos = (0, 0, G)") {
    ComCtrl ctrl;
    Vec3D ref_com_pos = {0, 0, G};
    double desired_zeta =
        ctrl.model().computeZeta(ref_com_pos, kVec3DZero, kVec3DZero);

    ctrl.x().set_q1(1.0).set_q2(0.5);
    ctrl.y().set_q1(1.2).set_q2(0.8);

    REQUIRE(desired_zeta == Approx(1.0));
    REQUIRE(ctrl.x().q1() == 1.0);
    REQUIRE(ctrl.x().q2() == 0.5);
    REQUIRE(ctrl.y().q1() == 1.2);
    REQUIRE(ctrl.y().q2() == 0.8);

    WHEN("com_pos = (1, 1, G), com_vel = (0, 0, 0)") {
      Vec3D com_pos = {1, 1, G};
      Vec3D com_vel = {0, 0, 0};

      THEN("desire_zmp_pos = (1.5, 1.96, 0)") {
        Vec3D expected_zmp_pos = {1.5, 1.96, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
    WHEN("com_pos = (-2, 3, G), com_vel = (2, -1, 0)") {
      Vec3D com_pos = {-2, 3, G};
      Vec3D com_vel = {2, -1, 0};

      THEN("desire_zmp_pos = (0, 3.88, 0)") {
        Vec3D expected_zmp_pos = {0, 3.88, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
  }
  GIVEN(
      "qx1 = 1.0, qx2 = 1.5, qy1 = 1.0, qy2 = 1.0, ref_com_pos = (1, 0.5, "
      "G)") {
    ComCtrl ctrl;
    Vec3D ref_com_pos = {1, 0.5, G};
    double desired_zeta =
        ctrl.model().computeZeta(ref_com_pos, kVec3DZero, kVec3DZero);

    ctrl.x().set_q2(1.5);

    REQUIRE(desired_zeta == Approx(1.0));
    REQUIRE(ctrl.x().q1() == 1);
    REQUIRE(ctrl.x().q2() == 1.5);
    REQUIRE(ctrl.y().q1() == 1);
    REQUIRE(ctrl.y().q2() == 1);

    WHEN("com_pos = (1, 0, G), com_vel = (0, 0, 0)") {
      Vec3D com_pos = {1, 0, G};
      Vec3D com_vel = {0, 0, 0};

      THEN("desire_zmp_pos = (1, -0.5, 0)") {
        Vec3D expected_zmp_pos = {1, -0.5, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
    WHEN("com_pos = (0, 3, G), com_vel = (-2, -1, 0)") {
      Vec3D com_pos = {0, 3, G};
      Vec3D com_vel = {-2, -1, 0};

      THEN("desire_zmp_pos = (-6.5, 3.5, 0)") {
        Vec3D expected_zmp_pos = {-6.5, 3.5, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
    WHEN("com_pos = (-2, 0, G), com_vel = (3, -2, 0)") {
      Vec3D com_pos = {-2, 0, G};
      Vec3D com_vel = {3, -2, 0};

      THEN("desire_zmp_pos = (1, -4.5, 0)") {
        Vec3D expected_zmp_pos = {1, -4.5, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
  }

  GIVEN(
      "qx1 = 1.0, qx2 = 1.5, qy1 = 1.0, qy2 = 1.5, "
      "ref_com_pos = (0, 0.5, 0.5*G)") {
    ComCtrl ctrl;
    Vec3D ref_com_pos = {0, 0.5, 0.5 * G};
    double desired_zeta =
        ctrl.model().computeZeta(ref_com_pos, kVec3DZero, kVec3DZero);

    ctrl.x().set_q2(1.5);
    ctrl.y().set_q2(1.5);

    REQUIRE(desired_zeta == Approx(sqrt(2.0)));
    REQUIRE(ctrl.x().q1() == 1);
    REQUIRE(ctrl.x().q2() == 1.5);
    REQUIRE(ctrl.y().q1() == 1);
    REQUIRE(ctrl.y().q2() == 1.5);

    WHEN("com_pos = (0, 0, G), com_vel = (0, 0, 0)") {
      Vec3D com_pos = {0, 0, G};
      Vec3D com_vel = {0, 0, 0};

      THEN("desire_zmp_pos = (0, -0.75, 0)") {
        Vec3D expected_zmp_pos = {0, -0.75, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
    WHEN("com_pos = (1, 3, G), com_vel = (0, -2, 0)") {
      Vec3D com_pos = {1, 3, G};
      Vec3D com_vel = {0, -2, 0};

      THEN("desire_zmp_pos = (2.5, 2.5*(3-sqrt(2))-0.75, 0)") {
        Vec3D expected_zmp_pos = {2.5, 2.5 * (3 - sqrt(2)) - 0.75, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
      }
    }
    WHEN("com_pos = (-2, 0, G), com_vel = (3, -1, 0)") {
      Vec3D com_pos = {-2, 0, G};
      Vec3D com_vel = {3, -1, 0};

      THEN("desire_zmp_pos = (2.5*(-2+1.5*sqrt(2)), -1.25*sqrt(2)-0.75, 0)") {
        Vec3D expected_zmp_pos = {2.5 * (-2 + 1.5 * sqrt(2)),
                                  -1.25 * sqrt(2) - 0.75, 0};
        Vec3D desired_zmp_pos =
            ctrl.computeDesZmpPos(ref_com_pos, com_pos, com_vel, desired_zeta);
        CHECK_THAT(desired_zmp_pos, Equals(expected_zmp_pos));
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
  auto cmd = ctrl.getUserCommands();

  struct testcase_t {
    Vec3D cmd_com_pos;
  } testcases[] = {{{0, 0, 1}}, {{0.1, 0, 1}}, {{0.1, -0.1, 1}}};
  for (auto& c : testcases) {
    Vec3D expected_des_zmp_pos;
    double expected_des_zeta;

    expected_des_zeta =
        ctrl.model().computeZeta(c.cmd_com_pos, kVec3DZero, kVec3DZero);
    expected_des_zmp_pos = ctrl.computeDesZmpPos(
        c.cmd_com_pos, ctrl.model().data()->com_position,
        ctrl.model().data()->com_velocity, expected_des_zeta);

    cmd->com_position = c.cmd_com_pos;
    ctrl.update();
    CHECK(ctrl.outputs().zeta == expected_des_zeta);
    CHECK_THAT(ctrl.outputs().zmp_position, Equals(expected_des_zmp_pos));
  }
}

SCENARIO("controller can regulate COM position at a point",
         "[corelib][humanoid]") {
  GIVEN("command that COM position be at (0.1, -0.1, 1)") {
    ComCtrl ctrl;
    Vec3D cmd_com_pos = {0.1, -0.1, 1};
    ctrl.getUserCommands()->com_position = cmd_com_pos;

    WHEN("at first") {
      THEN("COM position is at (0, 0, 1)") {
        Vec3D expected_com_pos = {0, 0, 1};
        CHECK_THAT(ctrl.model().data()->com_position, Equals(expected_com_pos));
      }
    }
    WHEN("update until 0.1 sec") {
      double t = 0;
      while (t < 0.1) {
        ctrl.update();
        t += ctrl.time_step();
      }
      THEN("COM position is between (0, 0, 1) and (0.1, -0.1, 1)") {
        Vec3D pos = ctrl.model().data()->com_position;
        CAPTURE(pos);
        CHECK(pos.x() > 0.0);
        CHECK(pos.x() < 0.1);
        CHECK(pos.y() < 0.0);
        CHECK(pos.y() > -0.1);
      }
    }
    WHEN("update until 10 sec") {
      double t = 0;
      while (t < 10) {
        ctrl.update();
        t += ctrl.time_step();
      }
      THEN("COM lies at (0.1, -0.1, 1)") {
        CHECK_THAT(ctrl.model().data()->com_position, Equals(cmd_com_pos));
      }
    }
  }
}

TEST_CASE("com_ctrl: when COM height is zero, update should fail",
          "[corelib][humanoid]") {
  ComCtrl ctrl;
  Vec3D p = {0, 0, 0};

  ctrl.model().data()->com_position = p;
  zEchoOff();
  CHECK_FALSE(ctrl.update());
  zEchoOn();
}

TEST_CASE("ComCtrl::update() when given COM height is zero, update should fail",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  Vec3D p = {0, 0, 0};

  ctrl.states().com_position = p;
  zEchoOff();
  CHECK_FALSE(ctrl.update());
  zEchoOn();
}

}  // namespace
}  // namespace holon
