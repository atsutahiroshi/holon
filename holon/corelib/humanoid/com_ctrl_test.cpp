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
#include "holon/corelib/humanoid/com_ctrl_x.hpp"
#include "holon/corelib/humanoid/com_ctrl_y.hpp"
#include "holon/corelib/humanoid/com_ctrl_z.hpp"
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

#include "catch.hpp"
#include "holon/test/util/catch/custom_matchers.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

using Catch::Matchers::Equals;

namespace ctrl_x = com_ctrl_x;
namespace ctrl_y = com_ctrl_y;
namespace ctrl_z = com_ctrl_z;

const double G = RK_G;

TEST_CASE("ComCtrlCommands::clear() should clear all the values") {
  Fuzzer fuzz;
  ComCtrlCommands cmd;

  // randomize all
  cmd.xd = fuzz.get();
  cmd.yd = fuzz.get();
  cmd.zd = fuzz.get();
  cmd.vxd = fuzz.get();
  cmd.vyd = fuzz.get();
  cmd.qx1 = fuzz.get();
  cmd.qx2 = fuzz.get();
  cmd.qy1 = fuzz.get();
  cmd.qy2 = fuzz.get();
  cmd.qz1 = fuzz.get();
  cmd.qz2 = fuzz.get();
  cmd.rho = fuzz.get();
  cmd.dist = fuzz.get();
  cmd.kr = fuzz.get();
  cmd.vhp = fuzz.get();

  // clear
  cmd.clear();

  // check
  CHECK(cmd.xd == nullopt);
  CHECK(cmd.yd == nullopt);
  CHECK(cmd.zd == nullopt);
  CHECK(cmd.vxd == nullopt);
  CHECK(cmd.vyd == nullopt);
  CHECK(cmd.qx1 == nullopt);
  CHECK(cmd.qx2 == nullopt);
  CHECK(cmd.qy1 == nullopt);
  CHECK(cmd.qy2 == nullopt);
  CHECK(cmd.qz1 == nullopt);
  CHECK(cmd.qz2 == nullopt);
  CHECK(cmd.rho == nullopt);
  CHECK(cmd.dist == nullopt);
  CHECK(cmd.kr == nullopt);
  CHECK(cmd.vhp == nullopt);
}

TEST_CASE("ComCtrlRefs: constructor", "[ComCtrlRefs]") {
  SECTION("default constructor") {
    ComCtrlRefs refs;
    CHECK(refs.com_position == ComZmpModelData::default_com_position);
    CHECK(refs.com_velocity == kVec3DZero);
    CHECK(refs.qx1 == ctrl_x::default_q1);
    CHECK(refs.qx2 == ctrl_x::default_q2);
    CHECK(refs.qy1 == ctrl_y::default_q1);
    CHECK(refs.qy2 == ctrl_y::default_q2);
    CHECK(refs.rho == ctrl_y::default_rho);
    CHECK(refs.dist == ctrl_y::default_dist);
    CHECK(refs.kr == ctrl_y::default_kr);
    CHECK(refs.qz1 == ctrl_z::default_q1);
    CHECK(refs.qz2 == ctrl_z::default_q2);
    CHECK(refs.vhp == 0);
  }

  SECTION("const ComZmpModelData&") {
    Fuzzer fuzz(0, 5);
    auto p0 = fuzz.get<Vec3D>();
    auto data = createComZmpModelData(p0);
    ComCtrlRefs refs(*data);
    CHECK(refs.com_position == p0);
    CHECK(refs.com_velocity == kVec3DZero);
    CHECK(refs.qx1 == ctrl_x::default_q1);
    CHECK(refs.qx2 == ctrl_x::default_q2);
    CHECK(refs.qy1 == ctrl_y::default_q1);
    CHECK(refs.qy2 == ctrl_y::default_q2);
    CHECK(refs.rho == ctrl_y::default_rho);
    CHECK(refs.dist == ctrl_y::default_dist);
    CHECK(refs.kr == ctrl_y::default_kr);
    CHECK(refs.qz1 == ctrl_z::default_q1);
    CHECK(refs.qz2 == ctrl_z::default_q2);
    CHECK(refs.vhp == 0);
  }

  SECTION("const ComZmpModel&") {
    Fuzzer fuzz(0, 5);
    auto p0 = fuzz.get<Vec3D>();
    ComZmpModel model(p0);
    ComCtrlRefs refs(model);
    CHECK(refs.com_position == p0);
    CHECK(refs.com_velocity == kVec3DZero);
    CHECK(refs.qx1 == ctrl_x::default_q1);
    CHECK(refs.qx2 == ctrl_x::default_q2);
    CHECK(refs.qy1 == ctrl_y::default_q1);
    CHECK(refs.qy2 == ctrl_y::default_q2);
    CHECK(refs.rho == ctrl_y::default_rho);
    CHECK(refs.dist == ctrl_y::default_dist);
    CHECK(refs.kr == ctrl_y::default_kr);
    CHECK(refs.qz1 == ctrl_z::default_q1);
    CHECK(refs.qz2 == ctrl_z::default_q2);
    CHECK(refs.vhp == 0);
  }
}

TEST_CASE("ComCtrl: constructor", "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  SECTION("check if member pointers are preserved") {
    REQUIRE(&ctrl.states());
    REQUIRE(&ctrl.refs());
    REQUIRE(&ctrl.outputs());
    REQUIRE(&ctrl.commands());
    REQUIRE(&ctrl.model().data() == &ctrl.states());
  }

  SECTION("check if canonical foot distance be initialized") {
    CHECK(ctrl.canonical_foot_dist() == ctrl_y::default_dist);
  }
}

TEST_CASE("ComCtrl(const Model&) constructor", "[corelib][humanoid][ComCtrl]") {
  ComCtrl::Model model;
  Fuzzer fuzz;
  auto p = fuzz.get<Vec3D>();
  auto v = fuzz.get<Vec3D>();
  model.data_ptr()->com_position = p;
  model.data_ptr()->com_velocity = v;

  ComCtrl ctrl(model);
  CHECK(ctrl.model().initial_com_position() == p);
  CHECK(ctrl.states().com_position == p);
  CHECK(ctrl.states().com_velocity == v);

  p = fuzz.get<Vec3D>();
  v = fuzz.get<Vec3D>();
  ctrl.states().com_position = p;
  ctrl.states().com_velocity = v;
  CHECK(model.data().com_position != p);
  CHECK(model.data().com_velocity != v);
  CHECK(ctrl.model().initial_com_position() != p);
  CHECK(ctrl.states().com_position == p);
  CHECK(ctrl.states().com_velocity == v);
}

TEST_CASE("ComCtrl::set_states_ptr can set another pointer to states data",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  {
    auto states1 = createComZmpModelData(Vec3D(0, 0, 1), 3);
    REQUIRE(ctrl.states_ptr().get() != states1.get());
    ctrl.set_states_ptr(states1);
    REQUIRE(ctrl.states_ptr().get() == states1.get());
    REQUIRE(ctrl.states().mass == 3.0);
    REQUIRE(ctrl.model().data_ptr().get() == states1.get());
    REQUIRE(ctrl.model().mass() == 3.0);
    REQUIRE(states1.use_count() == 3);
  }
  REQUIRE(ctrl.states_ptr().use_count() == 2);
  ctrl.states_ptr()->mass = 5.0;
  REQUIRE(ctrl.states().mass == 5.0);
  REQUIRE(ctrl.model().mass() == 5.0);
}

TEST_CASE("ComCtrl::set_refs_ptr can set another pointer to refs data",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  {
    auto refs1 = createComCtrlRefs();
    REQUIRE(ctrl.refs_ptr().get() != refs1.get());
    ctrl.set_refs_ptr(refs1);
    REQUIRE(ctrl.refs_ptr().get() == refs1.get());
    REQUIRE(refs1.use_count() == 2);
  }
  REQUIRE(ctrl.refs_ptr().use_count() == 1);
}

TEST_CASE("ComCtrl::set_outputs_ptr can set another pointer to outputs data",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  {
    auto outputs1 = createComCtrlOutputs();
    REQUIRE(ctrl.outputs_ptr().get() != outputs1.get());
    ctrl.set_outputs_ptr(outputs1);
    REQUIRE(ctrl.outputs_ptr().get() == outputs1.get());
    REQUIRE(outputs1.use_count() == 2);
  }
  REQUIRE(ctrl.outputs_ptr().use_count() == 1);
}

TEST_CASE("ComCtrl::set_canonical_foot_dist sets canonical foot distance",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  double dist = Fuzzer(0, 1).get<double>();
  REQUIRE(ctrl.canonical_foot_dist() != dist);

  ctrl.set_canonical_foot_dist(dist);
  CHECK(ctrl.canonical_foot_dist() == dist);
}

TEST_CASE("ComCtrl::reset(const Vec3D&) should reset initial COM position",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  Vec3D p0 = {0, 0, 1.5};
  REQUIRE_THAT(ctrl.model().initial_com_position(), !Equals(p0));

  ctrl.reset(p0);
  CHECK_THAT(ctrl.model().initial_com_position(), Equals(p0));
  CHECK_THAT(ctrl.states().com_position, Equals(p0));
  CHECK_THAT(ctrl.states().com_velocity, Equals(kVec3DZero));
}

TEST_CASE(
    "ComCtrl::reset(const Vec3D&, double) resets initial COM position and foot "
    "distance") {
  ComCtrl ctrl;
  Vec3D p0 = {-0.1, 0.1, 1.5};
  double dist = Fuzzer(0, 1).get();
  REQUIRE_THAT(ctrl.model().initial_com_position(), !Equals(p0));
  REQUIRE(ctrl.canonical_foot_dist() != dist);

  ctrl.reset(p0, dist);
  CHECK_THAT(ctrl.model().initial_com_position(), Equals(p0));
  CHECK_THAT(ctrl.states().com_position, Equals(p0));
  CHECK_THAT(ctrl.states().com_velocity, Equals(kVec3DZero));
  CHECK(ctrl.canonical_foot_dist() == dist);
}

TEST_CASE("ComCtrl::getCommands() provides a pointer to user commands",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;

  auto cmd = ctrl.getCommands();
  CHECK(cmd->xd == nullopt);
  CHECK(cmd->xd != 0.0);
  CHECK_FALSE(cmd->xd);
  CHECK_FALSE(cmd->xd.has_value());
  CHECK_THROWS_AS(cmd->xd.value(), bad_optional_access);
  CHECK(cmd->xd.value_or(0.0) == 0.0);

  cmd->xd = 0.42;
  CHECK(cmd->xd == 0.42);
  CHECK(cmd->xd.value() == 0.42);
  CHECK(cmd->xd.value_or(0.0) == 0.42);
}

TEST_CASE("ComCtrl::feedback should copy COM position / velocity",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  Fuzzer fuzz;

  SECTION("ComCtrl::feedback(const Vec3D&,const Vec3D&)") {
    Vec3D p = fuzz.get<Vec3D>();
    Vec3D v = fuzz.get<Vec3D>();
    REQUIRE(ctrl.states().com_position != p);
    REQUIRE(ctrl.states().com_velocity != v);
    ctrl.feedback(p, v);
    CHECK(ctrl.states().com_position == p);
    CHECK(ctrl.states().com_velocity == v);
  }

  SECTION("ComCtrl::feedback(const Model::DataPtr&)") {
    Vec3D p = fuzz.get<Vec3D>();
    Vec3D v = fuzz.get<Vec3D>();
    auto data = createComZmpModelData();
    data->com_position = p;
    data->com_velocity = v;

    REQUIRE(ctrl.states().com_position != p);
    REQUIRE(ctrl.states().com_velocity != v);
    ctrl.feedback(data);
    CHECK(ctrl.states().com_position == p);
    CHECK(ctrl.states().com_velocity == v);
  }

  SECTION("ComCtrl::feedback(const Model&)") {
    Vec3D p = fuzz.get<Vec3D>();
    Vec3D v = fuzz.get<Vec3D>();
    ComCtrl::Model model;
    model.data_ptr()->com_position = p;
    model.data_ptr()->com_velocity = v;

    REQUIRE(ctrl.states().com_position != p);
    REQUIRE(ctrl.states().com_velocity != v);
    ctrl.feedback(model);
    CHECK(ctrl.states().com_position == p);
    CHECK(ctrl.states().com_velocity == v);
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
  auto cmd = ctrl.getCommands();

  struct testcase_t {
    Vec3D pd;
  } testcases[] = {{{0, 0, 1}}, {{0.1, 0, 1}}, {{0.1, -0.1, 1}}};
  for (auto& c : testcases) {
    Vec3D expected_pz;
    ctrl.refs_ptr()->com_position = c.pd;
    expected_pz = ctrl.computeDesZmpPos(ctrl.states().com_position,
                                        ctrl.states().com_velocity, 0);

    cmd->set_com_position(c.pd);
    ctrl.update();
    CHECK(ctrl.outputs().zmp_position == expected_pz);
  }
}

TEST_CASE("ComCtrl::update() updates control paramters",
          "[corelib][humanoid][ComCtrl]") {
  ComCtrl ctrl;
  Vec3D p0 = {0.1, -0.1, 1.5};
  double dist = 0.42;
  ctrl.reset(p0, dist);
  auto cmd = ctrl.getCommands();
  auto refs = ctrl.refs_ptr();

  Fuzzer fuzz;
  SECTION("COM position") {
    refs->com_position = fuzz.get<Vec3D>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().com_position == p0);
    }
    SECTION("set user command") {
      Vec3D v = {1.2, -1.2, 1.5};
      cmd->set_com_position(v);
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().com_position == v);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().com_position == p0);
    }
  }

  SECTION("COM velocity") {
    refs->com_velocity = fuzz.get<Vec3D>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().com_velocity == kVec3DZero);
    }
    SECTION("set user command") {
      Vec3D v = {1.2, -1.2, 0};
      cmd->set_com_velocity(v.x(), v.y());
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().com_velocity == v);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().com_velocity == kVec3DZero);
    }
  }

  SECTION("qx1 and qx2") {
    refs->qx1 = fuzz.get<double>();
    refs->qx2 = fuzz.get<double>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().qx1 == 1.0);
      CHECK(ctrl.refs().qx2 == 1.0);
    }
    SECTION("set user command") {
      cmd->qx1 = 0.8;
      cmd->qx2 = 0.5;
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().qx1 == 0.8);
      CHECK(ctrl.refs().qx2 == 0.5);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().qx1 == 1.0);
      CHECK(ctrl.refs().qx2 == 1.0);
    }
  }

  SECTION("qy1 and qy2") {
    refs->qy1 = fuzz.get<double>();
    refs->qy2 = fuzz.get<double>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().qy1 == 1.0);
      CHECK(ctrl.refs().qy2 == 1.0);
    }
    SECTION("set user command") {
      cmd->qy1 = 0.8;
      cmd->qy2 = 0.5;
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().qy1 == 0.8);
      CHECK(ctrl.refs().qy2 == 0.5);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().qy1 == 1.0);
      CHECK(ctrl.refs().qy2 == 1.0);
    }
  }

  SECTION("qz1 and qz2") {
    refs->qz1 = fuzz.get<double>();
    refs->qz2 = fuzz.get<double>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().qz1 == 1.0);
      CHECK(ctrl.refs().qz2 == 1.0);
    }
    SECTION("set user command") {
      cmd->qz1 = 0.8;
      cmd->qz2 = 0.5;
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().qz1 == 0.8);
      CHECK(ctrl.refs().qz2 == 0.5);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().qz1 == 1.0);
      CHECK(ctrl.refs().qz2 == 1.0);
    }
  }

  SECTION("Virtual horizontal plane") {
    refs->vhp = fuzz.get<double>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().vhp == 0.0);
    }
    SECTION("set user command") {
      cmd->vhp = 0.1;
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().vhp == 0.1);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().vhp == 0.0);
    }
  }

  SECTION("Stepping activation parameter") {
    refs->rho = fuzz();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().rho == 0.0);
    }
    SECTION("set user command") {
      cmd->rho = 1.0;
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().rho == 1.0);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().rho == 0.0);
    }
  }

  SECTION("Canonical foot distance") {
    refs->dist = fuzz();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().dist == dist);
    }
    SECTION("set user command") {
      cmd->dist = 1.0;
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().dist == 1.0);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().dist == dist);
    }
  }

  SECTION("initial energy exersion") {
    refs->kr = fuzz();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().kr == 1.0);
    }
    SECTION("set user command") {
      cmd->kr = 0.5;
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().kr == 0.5);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.refs().kr == 1.0);
    }
  }

  SECTION("New variable") {
    SECTION("default") {}
    SECTION("set user command") {}
    SECTION("clear") {}
  }
}

SCENARIO("controller can regulate COM position at a point",
         "[corelib][humanoid]") {
  GIVEN("command that COM position be at (0.1, -0.1, 1)") {
    ComCtrl ctrl;
    Vec3D cmd_com_pos = {0.1, -0.1, 1};
    ctrl.getCommands()->set_com_position(cmd_com_pos);

    WHEN("at first") {
      THEN("COM position is at (0, 0, 1)") {
        Vec3D expected_com_pos = {0, 0, 1};
        CHECK_THAT(ctrl.states().com_position, Equals(expected_com_pos));
      }
    }
    WHEN("update until 0.1 sec") {
      double t = 0;
      while (t < 0.1) {
        ctrl.update();
        t += ctrl.time_step();
      }
      THEN("COM position is between (0, 0, 1) and (0.1, -0.1, 1)") {
        Vec3D pos = ctrl.states().com_position;
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
        CHECK_THAT(ctrl.states().com_position, Equals(cmd_com_pos));
      }
    }
  }
}

SCENARIO("regulate COM position along vertical direction",
         "[corelib][humanoid][ComCtrl]") {
  GIVEN(
      "desired COM position = (0, 0, 0.42), "
      "initial COM position = (0, 0, 0.4)") {
    ComCtrl ctrl;
    Vec3D pd = {0, 0, 0.42};
    Vec3D p0 = {0, 0, 0.4};
    ctrl.reset(p0);
    ctrl.getCommands()->set_com_position(pd);
    REQUIRE(ctrl.states().com_position == p0);
    REQUIRE(ctrl.commands().zd == 0.42);

    WHEN("update until 0.1 sec") {
      double t = 0;
      while (t < 0.1) {
        ctrl.update();
        t += ctrl.time_step();
      }
      THEN("COM position should be between (0,0,0.4) and (0,0,0.42)") {
        CAPTURE(ctrl.states().com_position);
        CHECK(ctrl.states().com_position.z() > 0.4);
        CHECK(ctrl.states().com_position.z() < 0.42);
      }
    }
    WHEN("update until 10 sec") {
      double t = 0;
      while (t < 10) {
        ctrl.update();
        t += ctrl.time_step();
      }
      THEN("COM position should be at (0,0,0.42)") {
        CAPTURE(ctrl.states().com_position);
        CHECK_THAT(ctrl.states().com_position, Equals(pd));
      }
    }
  }
}

TEST_CASE("com_ctrl: when COM height is zero, update should fail",
          "[corelib][humanoid]") {
  ComCtrl ctrl;
  Vec3D p = {0, 0, 0};

  ctrl.states().com_position = p;
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

SCENARIO("Controller makes COM oscillate sideward",
         "[corelib][humanoid][ComCtrl]") {
  GIVEN("Referential COM is at (0, 0, 0.42)") {
    ComCtrl ctrl;
    Vec3D pd = {0, 0, 0.42};
    double dist = 0.5;
    ctrl.reset(pd, dist);
    auto cmd = ctrl.getCommands();
    cmd->set_com_position(pd);
    cmd->dist = dist;

    WHEN("With rho = 0, update until 5 sec") {
      cmd->rho = 0;
      double t = 0;
      while (t < 5) {
        ctrl.update();
        t += ctrl.time_step();
      }
      THEN("No oscillation happens") {
        CHECK(ctrl.states().com_position == pd);
      }
    }
    WHEN("With rho = 1, update until 5 sec") {
      cmd->rho = 1;
      double t = 0;
      double yzmax = 0, yzmin = 0;
      ctrl.states().com_velocity[1] += 0.0001;
      while (t < 5) {
        ctrl.update();
        if (ctrl.states().zmp_position.y() > yzmax)
          yzmax = ctrl.states().zmp_position.y();
        if (ctrl.states().zmp_position.y() < yzmin)
          yzmin = ctrl.states().zmp_position.y();
        t += ctrl.time_step();
      }
      THEN("Oscillates with amplitude of 0.5") {
        CAPTURE(yzmax);
        CAPTURE(yzmin);
        CHECK((yzmax - yzmin) == Approx(dist));
        // FAIL(yzmax - yzmin);
      }
    }
  }
}

SCENARIO("ComCtrl::computeDesZmpPos computes desired ZMP position",
         "[corelib][humanoid][ComCtrl]") {
  double t = 0;
  GIVEN("qx1 = 1.0, qx2 = 1.0, qy1 = 1.0, qy2 = 1.0, ref_com_pos = (0, 0, G)") {
    ComCtrl ctrl;
    Vec3D pd = {0, 0, G};
    ctrl.refs_ptr()->com_position = pd;

    REQUIRE(ctrl.refs().qx1 == 1);
    REQUIRE(ctrl.refs().qx2 == 1);
    REQUIRE(ctrl.refs().qy1 == 1);
    REQUIRE(ctrl.refs().qy2 == 1);

    WHEN("p = (0, 0, G), v = (0, 0, 0)") {
      Vec3D p = {0, 0, G};
      Vec3D v = {0, 0, 0};

      THEN("desired_pz = (0, 0, 0)") {
        Vec3D expected_pz = {0, 0, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
    WHEN("p = (1, 3, G), v = (0, -1, 0)") {
      Vec3D p = {1, 3, G};
      Vec3D v = {0, -1, 0};

      THEN("desired_pz = (2, 4, 0)") {
        Vec3D expected_pz = {2, 4, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
    WHEN("p = (0, -2, G), v = (-2, 2, 0)") {
      Vec3D p = {0, -2, G};
      Vec3D v = {-2, 2, 0};

      THEN("desired_pz = (-4, 0, 0)") {
        Vec3D expected_pz = {-4, 0, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
  }

  GIVEN("qx1 = 1, qx2 = 0.5, qy1 = 1.2, qy2 = 0.8, pd = (0, 0, G)") {
    ComCtrl ctrl;
    Vec3D pd = {0, 0, G};
    ctrl.refs_ptr()->com_position = pd;
    ctrl.refs_ptr()->qx1 = 1.0;
    ctrl.refs_ptr()->qx2 = 0.5;
    ctrl.refs_ptr()->qy1 = 1.2;
    ctrl.refs_ptr()->qy2 = 0.8;

    REQUIRE(ctrl.refs().qx1 == 1.0);
    REQUIRE(ctrl.refs().qx2 == 0.5);
    REQUIRE(ctrl.refs().qy1 == 1.2);
    REQUIRE(ctrl.refs().qy2 == 0.8);

    WHEN("p = (1, 1, G), v = (0, 0, 0)") {
      Vec3D p = {1, 1, G};
      Vec3D v = {0, 0, 0};

      THEN("desired_pz = (1.5, 1.96, 0)") {
        Vec3D expected_pz = {1.5, 1.96, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
    WHEN("p = (-2, 3, G), v = (2, -1, 0)") {
      Vec3D p = {-2, 3, G};
      Vec3D v = {2, -1, 0};

      THEN("desired_pz = (0, 3.88, 0)") {
        Vec3D expected_pz = {0, 3.88, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
  }
  GIVEN(
      "qx1 = 1.0, qx2 = 1.5, qy1 = 1.0, qy2 = 1.0, "
      "pd = (1, 0.5, G)") {
    ComCtrl ctrl;
    Vec3D pd = {1, 0.5, G};
    ctrl.refs_ptr()->com_position = pd;
    ctrl.refs_ptr()->qx2 = 1.5;

    REQUIRE(ctrl.refs().qx1 == 1);
    REQUIRE(ctrl.refs().qx2 == 1.5);
    REQUIRE(ctrl.refs().qy1 == 1);
    REQUIRE(ctrl.refs().qy2 == 1);

    WHEN("p = (1, 0, G), v = (0, 0, 0)") {
      Vec3D p = {1, 0, G};
      Vec3D v = {0, 0, 0};

      THEN("desired_pz = (1, -0.5, 0)") {
        Vec3D expected_pz = {1, -0.5, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
    WHEN("p = (0, 3, G), v = (-2, -1, 0)") {
      Vec3D p = {0, 3, G};
      Vec3D v = {-2, -1, 0};

      THEN("desired_pz = (-6.5, 3.5, 0)") {
        Vec3D expected_pz = {-6.5, 3.5, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
    WHEN("p = (-2, 0, G), v = (3, -2, 0)") {
      Vec3D p = {-2, 0, G};
      Vec3D v = {3, -2, 0};

      THEN("desired_pz = (1, -4.5, 0)") {
        Vec3D expected_pz = {1, -4.5, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
  }

  GIVEN(
      "qx1 = 1.0, qx2 = 1.5, qy1 = 1.0, qy2 = 1.5, "
      "pd = (0, 0.5, 0.5*G)") {
    ComCtrl ctrl;
    Vec3D pd = {0, 0.5, 0.5 * G};
    ctrl.refs_ptr()->com_position = pd;
    ctrl.refs_ptr()->qx2 = 1.5;
    ctrl.refs_ptr()->qy2 = 1.5;

    REQUIRE(ctrl.refs().qx1 == 1);
    REQUIRE(ctrl.refs().qx2 == 1.5);
    REQUIRE(ctrl.refs().qy1 == 1);
    REQUIRE(ctrl.refs().qy2 == 1.5);

    WHEN("p = (0, 0, 0.5*G), v = (0, 0, 0)") {
      Vec3D p = {0, 0, 0.5 * G};
      Vec3D v = {0, 0, 0};

      THEN("desired_pz = (0, -0.75, 0)") {
        Vec3D expected_pz = {0, -0.75, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }

    WHEN("p = (1, 3, 0.5*G), v = (0, -2, 0)") {
      Vec3D p = {1, 3, 0.5 * G};
      Vec3D v = {0, -2, 0};

      THEN("desired_pz = (2.5, 2.5*(3-sqrt(2))-0.75, 0)") {
        Vec3D expected_pz = {2.5, 2.5 * (3 - sqrt(2)) - 0.75, 0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
    WHEN("p = (-2, 0, 0.5*G), v = (3, -1, 0)") {
      Vec3D p = {-2, 0, 0.5 * G};
      Vec3D v = {3, -1, 0};

      THEN("desired_pz = (2.5*(-2+1.5*sqrt(2)), -1.25*sqrt(2)-0.75, 0)") {
        Vec3D expected_pz = {2.5 * (-2 + 1.5 * sqrt(2)), -1.25 * sqrt(2) - 0.75,
                             0};
        CHECK(ctrl.computeDesZmpPos(p, v, t) == expected_pz);
      }
    }
  }
}

}  // namespace
}  // namespace holon
