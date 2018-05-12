/* com_controller - COM controller class
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of holon.
 *
 * Holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "holon2/corelib/humanoid/com_controller.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

const double kDefaultTimeStep = 0.001;
const double kTOL = 1e-10;
using ::Catch::Matchers::ApproxEquals;

ComZmpModel getRandomModel() {
  double mass = Random<double>(0, 2).get();
  double vhp = Random<double>(0, 1).get();
  Vec3d p0 = Random<Vec3d>(-1, 1).get();
  p0[2] += 2;  // make COM height be larger than vhp
  ComZmpModelBuilder b;
  b.setMass(mass).setVirtualHorizontalPlane(vhp).setComPosition(p0);
  return b.build();
}

void checkCtor(const ComController& ctrl, const double expected_mass,
               const Vec3d& expected_com_position) {
  CHECK(ctrl.data().subdata<0, 1>() == ctrl.model().data());
  CHECK(ctrl.mass() == expected_mass);
  CHECK(ctrl.params().com_position == expected_com_position);
  CHECK(ctrl.params().com_velocity == kVec3dZero);
  for (auto i = 0; i < 3; ++i) {
    CHECK(ctrl.params().q1[i] == ComController::default_q1[i]);
    CHECK(ctrl.params().q2[i] == ComController::default_q2[i]);
  }
  CHECK(ctrl.params().rho == ComController::default_rho);
  CHECK(ctrl.params().dist == ComController::default_dist);
  CHECK(ctrl.params().kr == ComController::default_kr);
  CHECK(ctrl.getInitialComPosition() == expected_com_position);
}
TEST_CASE("com_controller: check c'tors", "[ComController]") {
  using CZ = ComZmpModelBuilder;
  SECTION("default c'tor") {
    ComController ctrl;
    checkCtor(ctrl, CZ::default_mass, CZ::default_com_position);
  }
  SECTION("overloaded c'tor: with Data") {
    ComControllerData data;
    ComController ctrl(data);
    checkCtor(ctrl, CZ::default_mass, CZ::default_com_position);
    CHECK(ctrl.data() == data);
  }
  SECTION("overloaded c'tor: with Model") {
    const auto model = getRandomModel();
    ComController ctrl(model);
    checkCtor(ctrl, model.mass(), model.com_position());
    CHECK(ctrl.data().subdata<0, 1>() != model.data());
  }
}

TEST_CASE("com_controller: accessor to model parameters", "[ComController]") {
  ComControllerData data;
  auto& params = data.get<0>();
  Random<double> rnd;
  params.mass = rnd();
  params.vhp = rnd();
  ComController ctrl(data);
  CHECK(ctrl.mass() == params.mass);
  CHECK(ctrl.vhp() == params.vhp);
}

TEST_CASE("com_controller: accessor to model states", "[ComController]") {
  ComControllerData data;
  auto& states = data.get<1>();
  Random<Vec3d> rnd;
  states.com_position = rnd();
  states.com_velocity = rnd();
  states.zmp_position = rnd();
  states.contact_force = rnd();
  states.reaction_force = rnd();
  const ComController ctrl(data);
  CHECK(ctrl.states().com_position == states.com_position);
  CHECK(ctrl.states().com_velocity == states.com_velocity);
  CHECK(ctrl.states().zmp_position == states.zmp_position);
  CHECK(ctrl.states().contact_force == states.contact_force);
  CHECK(ctrl.states().reaction_force == states.reaction_force);
}

TEST_CASE("com_controller: accessor to control parameters", "[ComController]") {
  ComControllerData data;
  auto& params = data.get<2>();
  Random<double> rnd;
  params.q1 = {rnd(), rnd(), rnd()};
  params.q2 = {rnd(), rnd(), rnd()};
  const ComController ctrl(data);
  for (auto i = 0; i < 3; ++i) {
    CHECK(ctrl.params().q1[i] == params.q1[i]);
    CHECK(ctrl.params().q2[i] == params.q2[i]);
  }
}

TEST_CASE("com_controller: accessor to control outputs", "[ComController]") {
  ComControllerData data;
  auto& outputs = data.get<3>();
  Random<Vec3d> rnd;
  outputs.com_position = rnd();
  outputs.com_velocity = rnd();
  outputs.zmp_position = rnd();
  outputs.contact_force = rnd();
  const ComController ctrl(data);
  CHECK(ctrl.outputs().com_position == outputs.com_position);
  CHECK(ctrl.outputs().com_velocity == outputs.com_velocity);
  CHECK(ctrl.outputs().zmp_position == outputs.zmp_position);
  CHECK(ctrl.outputs().contact_force == outputs.contact_force);
}

TEST_CASE("com_controller: set time step", "[ComController]") {
  ComController ctrl;
  REQUIRE(ctrl.time_step() == kDefaultTimeStep);
  auto dt = Random<double>(0, 0.01).get();
  ctrl.setTimeStep(dt);
  CHECK(ctrl.time_step() == dt);
}

TEST_CASE("com_controller: update current time", "[ComController]") {
  ComController ctrl;
  Random<double> rnd(0, 0.01);
  SECTION("update with default time step") {
    REQUIRE(ctrl.time() == 0.0);
    ctrl.update();
    CHECK(ctrl.time() == kDefaultTimeStep);
    ctrl.update();
    CHECK(ctrl.time() == 2 * kDefaultTimeStep);
  }
  SECTION("modify time step") {
    auto dt = rnd();
    REQUIRE(ctrl.time() == 0.0);
    ctrl.setTimeStep(dt);
    ctrl.update();
    CHECK(ctrl.time() == dt);
    ctrl.update();
    CHECK(ctrl.time() == 2 * dt);
  }
  SECTION("modify time step temporariliy") {
    auto dt1 = rnd();
    REQUIRE(ctrl.time() == 0.0);
    ctrl.update(dt1);
    CHECK(ctrl.time() == dt1);
    CHECK(ctrl.time_step() == kDefaultTimeStep);
    auto dt2 = rnd();
    ctrl.update(dt2);
    CHECK(ctrl.time() == dt1 + dt2);
    CHECK(ctrl.time_step() == kDefaultTimeStep);
  }
}

TEST_CASE("com_controller: reset time", "[ComController]") {
  ComController ctrl;
  ctrl.update();
  REQUIRE(ctrl.time() != 0.0);
  ctrl.reset();
  CHECK(ctrl.time() == 0.0);
}

TEST_CASE("com_controller: reset COM position at initial one",
          "[ComController]") {
  auto model = getRandomModel();
  const Vec3d p0 = model.com_position();
  Random<Vec3d> rnd(-0.1, 0.1);
  ComController ctrl(model);
  auto& states = const_cast<ComZmpModelStates&>(
      static_cast<const ComController&>(ctrl).states());
  states.com_position = p0 + rnd();
  states.com_velocity = rnd();
  REQUIRE(ctrl.model().com_position() != p0);
  REQUIRE(ctrl.model().com_velocity() != kVec3dZero);
  ctrl.reset();
  CHECK(ctrl.model().com_position() == p0);
  CHECK(ctrl.model().com_velocity() == kVec3dZero);
  CHECK(ctrl.getInitialComPosition() == p0);
  CHECK(const_cast<const ComController&>(ctrl).params().com_position == p0);
}

TEST_CASE("com_controller: reset COM position at a specific one",
          "[ComController]") {
  auto model = getRandomModel();
  const Vec3d p = Vec3d::Random();
  ComController ctrl(model);
  auto& states = const_cast<ComZmpModelStates&>(
      static_cast<const ComController&>(ctrl).states());
  states.com_velocity = Vec3d::Random();
  REQUIRE(ctrl.model().com_position() != p);
  REQUIRE(ctrl.model().com_velocity() != kVec3dZero);
  ctrl.reset(p);
  CHECK(ctrl.model().com_position() == p);
  CHECK(ctrl.model().com_velocity() == kVec3dZero);
  CHECK(ctrl.getInitialComPosition() == p);
  CHECK(const_cast<const ComController&>(ctrl).params().com_position == p);
}

TEST_CASE("com_controller: feedback states", "[ComController]") {
  Random<Vec3d> rnd(-0.1, 0.1);
  ComController ctrl;
  const Vec3d p = ctrl.model().com_position() + rnd();
  const Vec3d v = rnd();
  REQUIRE(ctrl.model().com_position() != p);
  REQUIRE(ctrl.model().com_velocity() != v);
  SECTION("overloaded function 1: with Model instance") {
    auto model =
        ComZmpModelBuilder().setComPosition(p).setComVelocity(v).build();
    ctrl.feedback(model);
    CHECK(ctrl.model().com_position() == p);
    CHECK(ctrl.model().com_velocity() == v);
  }
  SECTION("overloaded function 2: with Model data") {
    ComZmpModelData data;
    data.get<1>().com_position = p;
    data.get<1>().com_velocity = v;
    ctrl.feedback(data);
    CHECK(ctrl.model().com_position() == p);
    CHECK(ctrl.model().com_velocity() == v);
  }
}

TEST_CASE(
    "com_controller: check consistency between states and update after udpate",
    "[ComController]") {
  const auto model = getRandomModel();
  ComController ctrl(model);
  while (ctrl.time() < 0.1) ctrl.update();
  const auto& c = ctrl;
  CHECK(c.outputs().com_position == ctrl.model().com_position());
  CHECK(c.outputs().com_velocity == ctrl.model().com_velocity());
  CHECK(c.outputs().com_acceleration == ctrl.model().com_acceleration());
  CHECK(c.outputs().zmp_position == ctrl.model().zmp_position());
  CHECK(c.outputs().contact_force == ctrl.model().contact_force());
}

ComControllerParams& getParamsRef(const ComController& ctrl) {
  return const_cast<ComControllerParams&>(ctrl.params());
}
ComZmpModelStates& getStatesRef(const ComController& ctrl) {
  return const_cast<ComZmpModelStates&>(ctrl.states());
}
SCENARIO("com_controller: regulate COM position", "[ComController]") {
  GIVEN("initial position is (1, 1, 1.5)") {
    ComController ctrl;
    ctrl.reset(Vec3d(1, 1, 1.5));

    WHEN("desired position is given at (0, 0, 1) and update for 10 sec") {
      const Vec3d pd(0, 0, 1);
      auto& params = getParamsRef(ctrl);
      params.com_position = pd;
      while (ctrl.time() < 10) ctrl.update(0.01);

      THEN("COM moves and converges at the desired position") {
        CHECK_THAT(ctrl.model().com_position(), ApproxEquals(pd, kTOL));
        CHECK_THAT(ctrl.model().com_velocity(), ApproxEquals(kVec3dZero, kTOL));
      }
    }
  }
}

SCENARIO("com_controller: oscillate COM sideward", "[ComController]") {
  GIVEN("initial position is (0, 0, 0.42)") {
    ComController ctrl;
    auto& params = getParamsRef(ctrl);
    const Vec3d p0(0, 0, 0.42);
    const double dist = 0.5;
    ctrl.reset(p0);
    params.dist = dist;

    WHEN("given rho = 0 and update for 5 sec") {
      params.rho = 0;
      while (ctrl.time() < 5) ctrl.update(0.01);
      THEN("nothing happens") {
        CAPTURE(ctrl.model().com_position());
        CHECK(ctrl.model().com_position() == p0);
      }
    }

    WHEN("given rho = 1 and update for 5 sec") {
      params.rho = 1;
      double yzmax = 0, yzmin = 0;
      getStatesRef(ctrl).com_velocity[1] += 0.0001;
      while (ctrl.time() < 5) {
        ctrl.update(0.001);
        if (ctrl.model().zmp_position().y() > yzmax)
          yzmax = ctrl.model().zmp_position().y();
        if (ctrl.model().zmp_position().y() < yzmin)
          yzmin = ctrl.model().zmp_position().y();
      }
      THEN("oscillation with amplitude of 0.5 will be produced") {
        CAPTURE(yzmax);
        CAPTURE(yzmin);
        CHECK((yzmax - yzmin) == Approx(dist));
      }
    }
  }
}

SCENARIO("com_controller: moving longitudinally with oscillation",
         "[ComController]") {
  GIVEN("initial position is (0, 0, 0.42)") {
    ComController ctrl;
    auto& params = getParamsRef(ctrl);
    const Vec3d p0(0, 0, 0.42);
    const double dist = 0.4;
    const double vxd = 0.1;
    ctrl.reset(p0);
    params.dist = dist;

    WHEN("given referential velocity 0.1 and update for 0.1 sec") {
      params.com_velocity[0] = vxd;
      params.rho = 1;
      params.q1[0] = 0;
      while (ctrl.time() < 0.1) ctrl.update(0.01);
      THEN("start moving forward") {
        CHECK(ctrl.model().com_position().x() > 0.0);
        CHECK(ctrl.model().com_velocity().x() > 0.0);
        CHECK(ctrl.model().com_velocity().x() < vxd);
      }

      WHEN("update for 5 sec") {
        while (ctrl.time() < 5) ctrl.update(0.01);
        THEN("velocity converges at the referential one") {
          CHECK(ctrl.model().com_position().x() > 0.0);
          CHECK(ctrl.model().com_velocity().x() == Approx(vxd));
        }

        WHEN("referential velocity is set for 0 and update") {
          params.com_velocity[0] = 0;
          params.rho = 0;
          params.q1[0] = 1;
          const double x = ctrl.model().com_position()[0];
          params.com_position[0] = x;
          while (ctrl.time() < 10) ctrl.update(0.01);
          THEN("velocity converges at the referential one") {
            CHECK(ctrl.model().com_position().x() == Approx(x));
            CHECK(ctrl.model().com_velocity().x() == Approx(0).margin(kTOL));
          }
        }
      }
    }
  }
}

TEST_CASE("com_controller: ", "[ComController]") {}

}  // namespace
}  // namespace holon
