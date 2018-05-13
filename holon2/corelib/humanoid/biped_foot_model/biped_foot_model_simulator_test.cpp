/* biped_foot_model_simulator - Simple simulator of foot model dynamics
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

#include "holon2/corelib/humanoid/biped_foot_model/biped_foot_model_simulator.hpp"
#include "holon2/corelib/humanoid/biped_foot_model.hpp"

#include "holon2/corelib/common/random.hpp"
#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

const double kDefaultTimeStep = 0.001;
const Vec3d kDefaultPosition = Vec3d(0, 0, 0);
const double kTOL = 1e-10;
using ::Catch::Matchers::ApproxEquals;

void checkCtor(const BipedFootModelSimulator& sim, const Vec3d& expected_p0) {
  CHECK(sim.time() == 0.0);
  CHECK(sim.time_step() == kDefaultTimeStep);
  CHECK(sim.model().position() == expected_p0);
  CHECK(sim.getInitialPosition() == expected_p0);
}
TEST_CASE("biped_foot_model_simulator: check c'tors",
          "[BipedFootModel][BipedFootModelSimulator]") {
  Random<Vec3d> rnd;
  SECTION("default c'tor") {
    BipedFootModelSimulator sim;
    checkCtor(sim, kDefaultPosition);
  }
  SECTION("overloaded c'tor 1: with Data instance") {
    BipedFootModelData data;
    Vec3d v = rnd();
    data.get<1>().position = v;
    BipedFootModelSimulator sim(data);
    checkCtor(sim, v);
    CHECK(sim.model().data() == data);
  }
  SECTION("overloaded c'tor 2: with Model instance") {
    Vec3d v = rnd();
    auto model = BipedFootModelBuilder().setPosition(v).build();
    BipedFootModelSimulator sim(model);
    checkCtor(sim, v);
    CHECK(sim.model().data() != model.data());
  }
}

BipedFootModelStates& getStates(const BipedFootModel& model) {
  return const_cast<BipedFootModelStates&>(model.states());
}
BipedFootModelParams& getParams(const BipedFootModel& model) {
  return const_cast<BipedFootModelParams&>(model.params());
}
TEST_CASE("biped_foot_model_simulator: set initial position to the current one",
          "[BipedFootModel][BipedFootModelSimulator]") {
  Random<Vec3d> rnd;
  BipedFootModelSimulator sim;
  getStates(sim.model()).position = rnd();
  const Vec3d p = sim.model().position();
  REQUIRE(sim.getInitialPosition() != p);
  sim.setInitialPosition();
  CHECK(sim.getInitialPosition() == p);
}

TEST_CASE("biped_foot_model_simulator: set initial position to a specific one",
          "[BipedFootModel][BipedFootModelSimulator]") {
  Random<Vec3d> rnd;
  BipedFootModelSimulator sim;
  const Vec3d p0 = rnd();
  sim.setInitialPosition(p0);
  CHECK(sim.getInitialPosition() == p0);
}

BipedFootModel getRandomModel() {
  double mass = Random<double>(0, 2).get();
  Vec3d p0 = Random<Vec3d>(-1, 1).get();
  BipedFootModelBuilder b;
  b.setMass(mass).setPosition(p0);
  return b.build();
}

Vec3d calculateForceExample(const Vec3d& p, const Vec3d& v,
                            const double /* t */) {
  double k1 = 10;
  double k2 = 0.1;
  return k1 * p + k2 * v;
}

TEST_CASE("biped_foot_model_simulator: set force functor",
          "[BipedFootModel][BipedFootModelSimulator]") {
  Random<Vec3d> rnd;
  BipedFootModelSimulator sim;
  Vec3d p = rnd();
  Vec3d v = rnd();

  SECTION("default force") { CHECK(sim.getForce(p, v, 0) == kVec3dZero); }
  SECTION("set constant force") {
    Vec3d f = rnd();
    sim.setForce(f);
    CHECK(sim.getForce(p, v, 0) == f);
  }
  SECTION("set force functor") {
    sim.setForce(calculateForceExample);
    Vec3d f = calculateForceExample(p, v, 0);
    CHECK(sim.getForce(p, v, 0) == f);
  }
}

TEST_CASE("biped_foot_model_simulator: compute acceleration",
          "[BipedFootModel][BipedFootModelSimulator]") {
  auto model = getRandomModel();
  double mass = model.mass();
  Vec3d p = model.position();
  Vec3d v = model.velocity();
  BipedFootModelSimulator sim(model);

  SECTION("when nothing is given, accel. should be zero") {
    CHECK_THAT(sim.getAccel(p, v, 0), ApproxEquals(kVec3dZero, kTOL));
  }

  SECTION("when force is given, accel. should be computed") {
    sim.setForce(calculateForceExample);
    Vec3d f = calculateForceExample(p, v, 0);
    Vec3d expected_acc = f / mass;
    CHECK_THAT(sim.getAccel(p, v, 0), ApproxEquals(expected_acc, kTOL));
  }
}

TEST_CASE("biped_foot_model_simulator: update current time",
          "[BipedFootModel][BipedFootModelSimulator]") {
  BipedFootModelSimulator sim;
  Random<double> rnd(0, 0.01);
  SECTION("update with default time step") {
    REQUIRE(sim.time() == 0.0);
    sim.update();
    CHECK(sim.time() == kDefaultTimeStep);
    sim.update();
    CHECK(sim.time() == 2 * kDefaultTimeStep);
  }
  SECTION("modify time step") {
    auto dt = rnd();
    REQUIRE(sim.time() == 0.0);
    sim.setTimeStep(dt);
    sim.update();
    CHECK(sim.time() == dt);
    sim.update();
    CHECK(sim.time() == 2 * dt);
  }
  SECTION("modify time step temporariliy") {
    auto dt1 = rnd();
    REQUIRE(sim.time() == 0.0);
    sim.update(dt1);
    CHECK(sim.time() == dt1);
    CHECK(sim.time_step() == kDefaultTimeStep);
    auto dt2 = rnd();
    sim.update(dt2);
    CHECK(sim.time() == dt1 + dt2);
    CHECK(sim.time_step() == kDefaultTimeStep);
  }
}

TEST_CASE("biped_foot_model_simulator: reset time",
          "[BipedFootModel][BipedFootModelSimulator]") {
  BipedFootModelSimulator sim;
  sim.update();
  REQUIRE(sim.time() != 0.0);
  sim.reset();
  CHECK(sim.time() == 0.0);
}

TEST_CASE("biped_foot_model_simulator: reset position at initial one",
          "[BipedFootModel][BipedFootModelSimulator]") {
  Random<Vec3d> rnd;
  auto model = getRandomModel();
  const Vec3d p0 = model.position();
  BipedFootModelSimulator sim(model);
  getStates(sim.model()).position = rnd();
  getStates(sim.model()).velocity = rnd();
  REQUIRE(sim.model().position() != p0);
  REQUIRE(sim.model().velocity() != kVec3dZero);
  sim.reset();
  CHECK(sim.model().position() == p0);
  CHECK(sim.model().velocity() == kVec3dZero);
  CHECK(sim.getInitialPosition() == p0);
}

TEST_CASE("biped_foot_model_simulator: reset position at specific one",
          "[BipedFootModel][BipedFootModelSimulator]") {
  Random<Vec3d> rnd;
  auto model = getRandomModel();
  const Vec3d p = rnd();
  BipedFootModelSimulator sim(model);
  getStates(sim.model()).position = rnd();
  getStates(sim.model()).velocity = rnd();
  REQUIRE(sim.model().position() != p);
  REQUIRE(sim.model().velocity() != kVec3dZero);
  sim.reset(p);
  CHECK(sim.model().position() == p);
  CHECK(sim.model().velocity() == kVec3dZero);
  CHECK(sim.getInitialPosition() == p);
}

SCENARIO("biped_foot_model_simulator: check if point move after update",
         "[BipedFootModel][BipedFootModelSimulator]") {
  // prepare foot model in which position is set at (0, 0, 0)
  auto model = BipedFootModelBuilder().build();
  BipedFootModelSimulator sim(model);

  GIVEN("let position move left-forward and upward") {
    const Vec3d f(1, 1, 1);
    sim.setForce(f);
    REQUIRE_THAT(sim.model().position(), ApproxEquals(kVec3dZero, kTOL));
    REQUIRE_THAT(sim.model().velocity(), ApproxEquals(kVec3dZero, kTOL));

    WHEN("update once") {
      sim.update();

      THEN("velocity should be modified towards left-forward and upward") {
        const Vec3d v = sim.model().velocity();
        CHECK(v.x() > 0.0);
        CHECK(v.y() > 0.0);
        CHECK(v.z() > 0.0);
      }
      THEN("but position should still remain at the initial position") {
        const Vec3d p = sim.model().position();
        CHECK(p.x() == Approx(0.0).margin(1e-5));
        CHECK(p.y() == Approx(0.0).margin(1e-5));
        CHECK(p.z() == Approx(0.0).margin(1e-5));
      }
    }
    WHEN("update several times") {
      for (auto i = 0; i < 5; ++i) {
        sim.update(0.01);
      }

      THEN("velocity should be left-forward and upward") {
        const Vec3d v = sim.model().velocity();
        CHECK(v.x() > 0.0);
        CHECK(v.y() > 0.0);
        CHECK(v.z() > 0.0);
      }
      THEN("and position should also move left-forward and upward") {
        const Vec3d p = sim.model().position();
        CHECK(p.x() > 1e-3);
        CHECK(p.y() > 1e-3);
        CHECK(p.z() > 1e-3);
      }
    }
  }
}

TEST_CASE("biped_foot_model_simulator: ",
          "[BipedFootModel][BipedFootModelSimulator]") {}

}  // namespace
}  // namespace holon
