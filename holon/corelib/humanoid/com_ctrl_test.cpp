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
#include <memory>
#include "holon/corelib/humanoid/com_ctrl/com_ctrl_x.hpp"
#include "holon/corelib/humanoid/com_ctrl/com_ctrl_y.hpp"
#include "holon/corelib/humanoid/com_ctrl/com_ctrl_z.hpp"

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

TEST_CASE("ComCtrlCommandsRawData::clear() should clear all the values") {
  Fuzzer fuzz;
  ComCtrlCommandsRawData cmd;

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

TEST_CASE("Check c'tor of ComCtrlData", "[ComCtrlData]") {
  SECTION("default constructor") {
    ComCtrlData data;
    constexpr auto i = ComCtrlData::ParamsDataIndex::get<0>();
    CHECK(data.get<i>().com_position ==
          ComZmpModelRawData::default_com_position);
    CHECK(data.get<i>().com_velocity == kVec3DZero);
    CHECK(data.get<i>().mass == data.get<0>().mass);
    CHECK(data.get<i>().qx1 == ctrl_x::default_q1);
    CHECK(data.get<i>().qx2 == ctrl_x::default_q2);
    CHECK(data.get<i>().qy1 == ctrl_y::default_q1);
    CHECK(data.get<i>().qy2 == ctrl_y::default_q2);
    CHECK(data.get<i>().rho == ctrl_y::default_rho);
    CHECK(data.get<i>().dist == ctrl_y::default_dist);
    CHECK(data.get<i>().kr == ctrl_y::default_kr);
    CHECK(data.get<i>().qz1 == ctrl_z::default_q1);
    CHECK(data.get<i>().qz2 == ctrl_z::default_q2);
    CHECK(data.get<i>().vhp == 0);
  }
}

void CheckCtor_0() {
  ComCtrl ctrl;
  REQUIRE(ctrl.model().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
  CHECK(ctrl.model().system().is_set_zmp_position());
  CHECK(ctrl.canonical_foot_dist() == ctrl_y::default_dist);
  CHECK(ctrl.default_com_position() == ctrl.initial_com_position());
}

void CheckCtor_1() {
  auto model = make_model<ComCtrl::Model>();
  Fuzzer fuzz;
  double mass = fuzz();
  Vec3D p = fuzz.get<Vec3D>(), v = fuzz.get<Vec3D>();
  model.states().mass = mass;
  model.states().com_position = p;
  model.states().com_velocity = v;

  ComCtrl ctrl(model);
  REQUIRE(ctrl.model().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
  CHECK(ctrl.model().system().is_set_zmp_position());
  CHECK(ctrl.canonical_foot_dist() == ctrl_y::default_dist);
  CHECK(ctrl.default_com_position() == ctrl.initial_com_position());

  CHECK(ctrl.model().data().get_ptr<0>() != model.data().get_ptr<0>());
  CHECK(ctrl.model().initial_com_position() == p);
  CHECK(ctrl.states().com_position == p);
  CHECK(ctrl.states().com_velocity == v);
  CHECK(ctrl.params().mass == mass);

  p = fuzz.get<Vec3D>();
  v = fuzz.get<Vec3D>();
  ctrl.states().com_position = p;
  ctrl.states().com_velocity = v;
  CHECK(model.states().com_position != p);
  CHECK(model.states().com_velocity != v);
  CHECK(ctrl.model().initial_com_position() != p);
  CHECK(ctrl.states().com_position == p);
  CHECK(ctrl.states().com_velocity == v);
}

void CheckCtor_2() {
  auto data = make_data<ComCtrl::Data>();
  Fuzzer fuzz;
  Vec3D p = fuzz.get<Vec3D>(), v = fuzz.get<Vec3D>();
  data.get<0>().com_position = p;
  data.get<0>().com_velocity = v;

  ComCtrl ctrl(data);
  REQUIRE(ctrl.model().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
  CHECK(ctrl.model().system().is_set_zmp_position());
  CHECK(ctrl.canonical_foot_dist() == ctrl_y::default_dist);
  CHECK(ctrl.default_com_position() == ctrl.initial_com_position());

  CHECK(ctrl.model().data().get_ptr<0>() == data.get_ptr<0>());
  CHECK(ctrl.model().initial_com_position() == p);
  CHECK(ctrl.states().com_position == p);
  CHECK(ctrl.states().com_velocity == v);

  p = fuzz.get<Vec3D>();
  v = fuzz.get<Vec3D>();
  ctrl.states().com_position = p;
  ctrl.states().com_velocity = v;
  CHECK(data.get<0>().com_position == p);
  CHECK(data.get<0>().com_velocity == v);
  CHECK(ctrl.model().initial_com_position() != p);
  CHECK(ctrl.states().com_position == p);
  CHECK(ctrl.states().com_velocity == v);
}

void CheckCtor_3() {
  auto data = make_data<ComCtrl::Data>();
  Fuzzer fuzz;
  Vec3D p = fuzz.get<Vec3D>(), v = fuzz.get<Vec3D>();
  data.get<0>().com_position = p;
  data.get<0>().com_velocity = v;
  auto model_ptr = std::make_shared<ComZmpModel>();

  ComCtrl ctrl(data, model_ptr);
  REQUIRE(ctrl.data() == data);
  REQUIRE(ctrl.model().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
  REQUIRE(ctrl.model().data().get_ptr<0>() == model_ptr->data().get_ptr<>());
  REQUIRE(&ctrl.model() == model_ptr.get());
  CHECK(ctrl.model().system().is_set_zmp_position());
  CHECK(ctrl.canonical_foot_dist() == ctrl_y::default_dist);
  CHECK(ctrl.default_com_position() == ctrl.initial_com_position());

  CHECK(ctrl.model().data().get_ptr<0>() == data.get_ptr<0>());
  CHECK(ctrl.model().initial_com_position() == p);
  CHECK(ctrl.states().com_position == p);
  CHECK(ctrl.states().com_velocity == v);

  p = fuzz.get<Vec3D>();
  v = fuzz.get<Vec3D>();
  ctrl.states().com_position = p;
  ctrl.states().com_velocity = v;
  CHECK(data.get<0>().com_position == p);
  CHECK(data.get<0>().com_velocity == v);
  CHECK(ctrl.model().initial_com_position() != p);
  CHECK(ctrl.states().com_position == p);
  CHECK(ctrl.states().com_velocity == v);
}

TEST_CASE("Check c'tors of ComCtrl", "[ComCtrl][ctor]") {
  SECTION("Default c'tor") { CheckCtor_0(); }
  SECTION("Overloaded c'tor 1") { CheckCtor_1(); }
  SECTION("Overloaded c'tor 2") { CheckCtor_2(); }
  SECTION("Overloaded c'tor 3") { CheckCtor_3(); }
}

TEST_CASE("ComCtrl::set_canonical_foot_dist sets canonical foot distance",
          "[ComCtrl]") {
  ComCtrl ctrl;
  double dist = Fuzzer(0, 1).get<double>();
  REQUIRE(ctrl.canonical_foot_dist() != dist);

  ctrl.set_canonical_foot_dist(dist);
  CHECK(ctrl.canonical_foot_dist() == dist);
}

TEST_CASE("Check default COM position defined in ComCtrl", "[ComCtrl]") {
  Vec3D p0 = {0.1, 0.5, 0.42};
  ComZmpModel model;
  model.reset(p0);
  ComCtrl ctrl(model);
  auto cmd = ctrl.getCommands();
  SECTION("it should be initialized as the initial COM position") {
    CHECK(ctrl.default_com_position() == p0);
    CHECK(ctrl.default_com_position() == ctrl.initial_com_position());
  }
  SECTION("it should be equal to the COM position when resetting") {
    Vec3D p1 = {-0.1, 0.3, 0.4};
    ctrl.reset(p1);
    CHECK(ctrl.default_com_position() == p1);
    CHECK(ctrl.default_com_position() == ctrl.initial_com_position());
  }
  SECTION("element x should be updated during moving") {
    cmd->vxd = 0.1;
    for (int i = 0; i < 10; ++i) ctrl.update();
    CHECK(ctrl.default_com_position().x() != ctrl.initial_com_position().x());
    CHECK(ctrl.default_com_position().x() == ctrl.states().com_position.x());
  }
  SECTION("element z should be updated when zd is given") {
    cmd->zd = 0.4;
    REQUIRE(ctrl.default_com_position().z() != 0.4);
    ctrl.update();
    CHECK(ctrl.default_com_position().z() == 0.4);
  }
}

TEST_CASE("ComCtrl::reset(const Vec3D&) should reset initial COM position",
          "[ComCtrl]") {
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
  CHECK(ctrl.params().dist == dist);
}

TEST_CASE("ComCtrl::getCommands() provides a pointer to user commands",
          "[ComCtrl]") {
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
          "[ComCtrl]") {
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
    auto data = make_data<ComZmpModelData>();
    data.get().com_position = p;
    data.get().com_velocity = v;

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
    model.states().com_position = p;
    model.states().com_velocity = v;

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
    ctrl.params().com_position = c.pd;
    expected_pz = ctrl.computeDesZmpPos(ctrl.states().com_position,
                                        ctrl.states().com_velocity, 0);

    cmd->set_com_position(c.pd);
    ctrl.update();
    CHECK(ctrl.outputs().zmp_position == expected_pz);
  }
}

TEST_CASE("ComCtrl::update() updates control paramters", "[ComCtrl]") {
  ComCtrl ctrl;
  Vec3D p0 = {0.1, -0.1, 1.5};
  double dist = 0.42;
  ctrl.reset(p0, dist);
  auto cmd = ctrl.getCommands();
  auto params = ctrl.params();

  Fuzzer fuzz;
  SECTION("COM position") {
    params.com_position = fuzz.get<Vec3D>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().com_position == p0);
    }
    SECTION("set user command") {
      Vec3D v = {1.2, -1.2, 1.5};
      cmd->set_com_position(v);
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().com_position == v);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().com_position == p0);
    }
  }

  SECTION("COM velocity") {
    params.com_velocity = fuzz.get<Vec3D>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().com_velocity == kVec3DZero);
    }
    SECTION("set user command") {
      Vec3D v = {1.2, -1.2, 0};
      cmd->set_com_velocity(v.x(), v.y());
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().com_velocity == v);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().com_velocity == kVec3DZero);
    }
  }

  SECTION("qx1 and qx2") {
    params.qx1 = fuzz.get<double>();
    params.qx2 = fuzz.get<double>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().qx1 == 1.0);
      CHECK(ctrl.params().qx2 == 1.0);
    }
    SECTION("set user command") {
      cmd->qx1 = 0.8;
      cmd->qx2 = 0.5;
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().qx1 == 0.8);
      CHECK(ctrl.params().qx2 == 0.5);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().qx1 == 1.0);
      CHECK(ctrl.params().qx2 == 1.0);
    }
  }

  SECTION("qy1 and qy2") {
    params.qy1 = fuzz.get<double>();
    params.qy2 = fuzz.get<double>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().qy1 == 1.0);
      CHECK(ctrl.params().qy2 == 1.0);
    }
    SECTION("set user command") {
      cmd->qy1 = 0.8;
      cmd->qy2 = 0.5;
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().qy1 == 0.8);
      CHECK(ctrl.params().qy2 == 0.5);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().qy1 == 1.0);
      CHECK(ctrl.params().qy2 == 1.0);
    }
  }

  SECTION("qz1 and qz2") {
    params.qz1 = fuzz.get<double>();
    params.qz2 = fuzz.get<double>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().qz1 == 1.0);
      CHECK(ctrl.params().qz2 == 1.0);
    }
    SECTION("set user command") {
      cmd->qz1 = 0.8;
      cmd->qz2 = 0.5;
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().qz1 == 0.8);
      CHECK(ctrl.params().qz2 == 0.5);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().qz1 == 1.0);
      CHECK(ctrl.params().qz2 == 1.0);
    }
  }

  SECTION("Virtual horizontal plane") {
    params.vhp = fuzz.get<double>();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().vhp == 0.0);
    }
    SECTION("set user command") {
      cmd->vhp = 0.1;
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().vhp == 0.1);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().vhp == 0.0);
    }
  }

  SECTION("Stepping activation parameter") {
    params.rho = fuzz();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().rho == 0.0);
    }
    SECTION("set user command") {
      cmd->rho = 1.0;
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().rho == 1.0);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().rho == 0.0);
    }
  }

  SECTION("Canonical foot distance") {
    params.dist = fuzz();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().dist == dist);
    }
    SECTION("set user command") {
      cmd->dist = 1.0;
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().dist == 1.0);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().dist == dist);
    }
  }

  SECTION("initial energy exersion") {
    params.kr = fuzz();
    SECTION("default") {
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().kr == 1.0);
    }
    SECTION("set user command") {
      cmd->kr = 0.5;
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().kr == 0.5);
    }
    SECTION("clear") {
      cmd->clear();
      REQUIRE(ctrl.update());
      CHECK(ctrl.params().kr == 1.0);
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

SCENARIO("regulate COM position along vertical direction", "[ComCtrl]") {
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
          "[ComCtrl]") {
  ComCtrl ctrl;
  Vec3D p = {0, 0, 0};

  ctrl.states().com_position = p;
  zEchoOff();
  CHECK_FALSE(ctrl.update());
  zEchoOn();
}

SCENARIO("Controller makes COM oscillate sideward", "[ComCtrl]") {
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

SCENARIO("Check relations among rho, qx1 and vxd", "[ComCtrl]") {
  GIVEN("COM controller is given") {
    ComCtrl ctrl;
    Vec3D p0 = {0, 0, 0.42};
    double dist = 0.1;
    ctrl.reset(p0, dist);
    auto cmd = ctrl.getCommands();
    REQUIRE(ctrl.params().rho == 0);
    REQUIRE(ctrl.params().qx1 == 1);
    WHEN("Referential velocity is 0.0 and update") {
      cmd->vxd = 0;
      ctrl.update();
      THEN("Should be rho = 0, qx1 = 1") {
        CHECK(ctrl.params().rho == 0);
        CHECK(ctrl.params().qx1 == 1);
      }
    }
    WHEN("Referential velocity is 0.1 and update") {
      cmd->vxd = 0.1;
      cmd->qx1 = 0.8;
      ctrl.update();
      THEN("Should be rho = 1, qx1 = 0") {
        CHECK(ctrl.params().rho == 1);
        CHECK(ctrl.params().qx1 == 0);
        WHEN("Referential velocity get back to 0 and update") {
          cmd->vxd = 0;
          ctrl.update();
          THEN("Should be rho = 0, qx1 = 0.8") {
            CHECK(ctrl.params().rho == 0);
            CHECK(ctrl.params().qx1 == 0.8);
          }
        }
      }
    }
    WHEN("Referential velocity is -0.1 and update") {
      cmd->vxd = -0.1;
      cmd->qx1 = 0.8;
      ctrl.update();
      THEN("Should be rho = 1, qx1 = 0") {
        CHECK(ctrl.params().rho == 1);
        CHECK(ctrl.params().qx1 == 0);
        WHEN("Referential velocity get back to 0 and update") {
          cmd->vxd = 0;
          ctrl.update();
          THEN("Should be rho = 0, qx1 = 0.8") {
            CHECK(ctrl.params().rho == 0);
            CHECK(ctrl.params().qx1 == 0.8);
          }
        }
      }
    }
  }
}

SCENARIO("Controller enables logitudinal moving", "[ComCtrl]") {
  GIVEN("Referential velocity is 0.1") {
    ComCtrl ctrl;
    Vec3D pd = {0, 0, 0.42};
    double dist = 0.3;
    double vxd = 0.1;
    ctrl.reset(pd, dist);
    auto cmd = ctrl.getCommands();
    cmd->vxd = vxd;
    cmd->rho = 1;
    REQUIRE(ctrl.states().com_position.x() == 0);
    REQUIRE(ctrl.states().com_velocity.x() == 0);
    WHEN("Update until 0.1 sec") {
      while (ctrl.time() < 0.1) {
        ctrl.update();
      }
      THEN("Start moving forward") {
        CAPTURE(ctrl.params().com_position.x());
        CAPTURE(ctrl.states().com_position.x());
        REQUIRE(ctrl.params().com_position.x() > 0.0);
        CHECK(ctrl.states().com_position.x() > 0.0);
        CHECK(ctrl.states().com_velocity.x() > 0.0);
        CHECK(ctrl.states().com_velocity.x() < vxd);
      }
    }
    WHEN("Update until 5 sec") {
      while (ctrl.time() < 5) {
        ctrl.update();
      }
      THEN("Follow referential velocity") {
        REQUIRE(ctrl.params().com_position.x() > 0.0);
        CHECK(ctrl.states().com_position.x() > 0.0);
        CHECK(ctrl.states().com_velocity.x() == Approx(vxd));
      }
    }
    WHEN("After update until 5 sec, issue stop command") {
      while (ctrl.time() < 5) {
        ctrl.update();
      }
      cmd->vxd = 0;
      cmd->rho = 0;
      double x = ctrl.states().com_position.x();
      while (ctrl.time() < 8) {
        ctrl.update();
      }
      THEN("Stop") {
        REQUIRE(ctrl.params().com_position.x() == x);
        CHECK(ctrl.states().com_position.x() == Approx(x));
        CHECK(ctrl.states().com_velocity.x() == Approx(0).margin(1e-6));
      }
    }
  }
}

TEST_CASE("Check phases of both feet", "[ComCtrl]") {
  ComCtrl ctrl;
  Vec3D p0 = {0, 0, 0.42};
  double dist = 0.1;
  ctrl.reset(p0, dist);
  auto cmd = ctrl.getCommands();
  cmd->rho = 1;
  ctrl.states().com_velocity[1] += 0.000001;
  while (ctrl.time() < 3) {
    // update until producing a stable oscillation
    ctrl.update();
  }
  while (ctrl.states().zmp_position.y() < 0.025) {
    // update until ZMP comes onto left foot sole
    ctrl.update();
    if (ctrl.time() > 10) FAIL("oscillation not produced");
  }
  REQUIRE(ctrl.phaseLF() > 0);
  REQUIRE(ctrl.phaseLF() < 1);
  REQUIRE(ctrl.phaseRF() == 0);
  REQUIRE(ctrl.phaseRF() == 0);
  while (ctrl.states().zmp_position.y() > -0.025) {
    // update until ZMP comes onto right foot sole
    ctrl.update();
    if (ctrl.time() > 10) FAIL("oscillation not produced");
  }
  REQUIRE(ctrl.phaseLF() == 0);
  REQUIRE(ctrl.phaseLF() == 0);
  REQUIRE(ctrl.phaseRF() > 0);
  REQUIRE(ctrl.phaseRF() < 1);
}

TEST_CASE(
    "When command dist is given, canonical foot distance should be updated",
    "[ComCtrl][this]") {
  ComCtrl ctrl;
  double dist = 0.1;
  REQUIRE(ctrl.canonical_foot_dist() != dist);
  auto cmd = ctrl.getCommands();
  cmd->dist = dist;
  ctrl.update();
  CHECK(ctrl.canonical_foot_dist() == dist);
}

SCENARIO("Check relations among rho, yd, dist and vyd", "[ComCtrl]") {
  GIVEN("COM is at (0, 0, 0.42) and foot distance is 0.1") {
    ComCtrl ctrl;
    Vec3D p0 = {0, 0, 0.42};
    double dist = 0.1;
    ctrl.reset(p0, dist);
    auto cmd = ctrl.getCommands();
    ctrl.states().com_velocity[1] += 0.000001;
    REQUIRE(ctrl.params().rho == 0);
    REQUIRE(ctrl.params().com_position[1] == 0);
    REQUIRE(ctrl.params().dist == dist);
    WHEN("referential velocity is 0.1 and update") {
      cmd->vyd = 0.1;
      while (ctrl.time() < 1) ctrl.update();
      THEN("start moving leftwards") {
        CHECK(ctrl.params().rho == 1);
        CHECK(ctrl.params().com_position[1] > 0);
        CHECK(ctrl.params().dist > dist);
        WHEN("referential velocity get back to 0 and update") {
          cmd->vyd = 0;
          double yd = ctrl.params().com_position[1];
          ctrl.update();
          THEN("stop at that position") {
            CHECK(ctrl.params().rho == 0);
            CHECK(ctrl.params().com_position[1] == yd);
            CHECK(ctrl.params().dist == dist);
          }
        }
      }
    }
    WHEN("referential velocity is -0.1 and update") {
      cmd->vyd = -0.1;
      while (ctrl.time() < 1) ctrl.update();
      THEN("start moving rightwards") {
        CHECK(ctrl.params().rho == 1);
        CHECK(ctrl.params().com_position[1] < 0);
        CHECK(ctrl.params().dist > dist);
        WHEN("referential velocity get back to 0 and update") {
          cmd->vyd = 0;
          double yd = ctrl.params().com_position[1];
          ctrl.update();
          THEN("stop at that position") {
            CHECK(ctrl.params().rho == 0);
            CHECK(ctrl.params().com_position[1] == yd);
            CHECK(ctrl.params().dist == dist);
          }
        }
      }
    }
  }
}

}  // namespace
}  // namespace holon
