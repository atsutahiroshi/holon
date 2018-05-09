/* com_zmp_model_simulator - Simple simulator of the dynamics of COM-ZMP model
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

#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_simulator.hpp"
#include "holon2/corelib/humanoid/com_zmp_model.hpp"
#include "holon2/corelib/humanoid/const_defs.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

const double kDefaultTimeStep = 0.001;
const Vec3d kDefaultComPosition = Vec3d(0, 0, 1);
const double kTOL = 1e-10;
using ::Catch::Matchers::ApproxEquals;
namespace cz = com_zmp_model_formula;

void checkCtor(const ComZmpModelSimulator& sim, const Vec3d& expected_pg) {
  CHECK(sim.time() == 0.0);
  CHECK(sim.time_step() == kDefaultTimeStep);
  CHECK(sim.model().com_position() == expected_pg);
  CHECK(sim.getInitialComPosition() == expected_pg);
  CHECK(sim.getInputType() == ComZmpModelSimulator::InputType::kNotDetermined);
}
TEST_CASE("com_zmp_model_simulator: check c'tors",
          "[ComZmpModel][ComZmpModelSimulator]") {
  Random<Vec3d> rnd;
  SECTION("default c'tor") {
    ComZmpModelSimulator sim;
    checkCtor(sim, kDefaultComPosition);
  }
  SECTION("overloaded c'tor 1: with Data instance") {
    ComZmpModelData data;
    Vec3d v = rnd();
    data.get<1>().com_position = v;
    ComZmpModelSimulator sim(data);
    checkCtor(sim, v);
    CHECK(sim.model().data() == data);
  }
  SECTION("overloaded c'tor 2: with Model instance") {
    Vec3d v = rnd();
    auto model = ComZmpModelBuilder().setComPosition(v).build();
    ComZmpModelSimulator sim(model);
    checkCtor(sim, v);
    CHECK(sim.model().data() != model.data());
  }
}

TEST_CASE("com_zmp_model_simulator: set initial COM position",
          "[ComZmpModel][ComZmpModelSimulator]") {
  Random<Vec3d> rnd;
  ComZmpModelSimulator sim;
  Vec3d p0 = rnd();
  sim.setInitialComPosition(p0);
  CHECK(sim.getInitialComPosition() == p0);
}

TEST_CASE("com_zmp_model_simulator: set ZMP position as input",
          "[ComZmpModel][ComZmpModelSimulator]") {
  ComZmpModelSimulator sim;
  using Type = ComZmpModelSimulator::InputType;
  REQUIRE(sim.getInputType() == Type::kNotDetermined);
  REQUIRE_FALSE(sim.isZmpPosAsInput());
  REQUIRE_FALSE(sim.isReactForceAsInput());
  SECTION("method 1") {
    sim.setInputType(Type::kZmpPosition);
    CHECK(sim.getInputType() == Type::kZmpPosition);
    CHECK(sim.isZmpPosAsInput());
    CHECK_FALSE(sim.isReactForceAsInput());
  }
  SECTION("method 2") {
    sim.setZmpPosAsInput();
    CHECK(sim.getInputType() == Type::kZmpPosition);
    CHECK(sim.isZmpPosAsInput());
    CHECK_FALSE(sim.isReactForceAsInput());
  }
}

TEST_CASE("com_zmp_model_simulator: set reaction force as input",
          "[ComZmpModel][ComZmpModelSimulator]") {
  ComZmpModelSimulator sim;
  using Type = ComZmpModelSimulator::InputType;
  REQUIRE(sim.getInputType() == Type::kNotDetermined);
  REQUIRE_FALSE(sim.isZmpPosAsInput());
  REQUIRE_FALSE(sim.isReactForceAsInput());
  SECTION("method 1") {
    sim.setInputType(Type::kReactionForce);
    CHECK(sim.getInputType() == Type::kReactionForce);
    CHECK(sim.isReactForceAsInput());
    CHECK_FALSE(sim.isZmpPosAsInput());
  }
  SECTION("method 2") {
    sim.setReactForceAsInput();
    CHECK(sim.getInputType() == Type::kReactionForce);
    CHECK(sim.isReactForceAsInput());
    CHECK_FALSE(sim.isZmpPosAsInput());
  }
}

ComZmpModel getRandomModel() {
  double mass = Random<double>(0, 2).get();
  double vhp = Random<double>(0, 1).get();
  Vec3d p0 = Random<Vec3d>(-1, 1).get();
  p0[2] += 2;  // make COM height be larger than vhp
  ComZmpModelBuilder b;
  b.setMass(mass).setVirtualHorizontalPlane(vhp).setComPosition(p0);
  return b.build();
}

Vec3d calculateZmpExample(const Vec3d& p, const Vec3d& v,
                          const double /* t */) {
  double k1 = 10;
  double k2 = 0.1;
  Vec3d pz = p + k1 * p + k2 * v;
  pz.z() = 0;
  return pz;
}
TEST_CASE("com_zmp_model_simulator: set ZMP position functor",
          "[ComZmpModel][ComZmpModelSimulator]") {
  auto model = getRandomModel();
  ComZmpModelSimulator sim(model);
  Vec3d p = Vec3d::Random();
  Vec3d v = Vec3d::Random();

  SECTION("default ZMP position") {
    Vec3d p0 = model.com_position();
    double vhp = model.vhp();
    CHECK(sim.getZmpPosition(p, v, 0) == Vec3d(p0[0], p0[1], vhp));
  }
  SECTION("set constant ZMP position") {
    Vec3d pz = Vec3d::Random();
    pz[2] = 0;
    sim.setZmpPosition(pz);
    CHECK(sim.getZmpPosition(p, v, 0) == pz);
  }
  SECTION("set ZMP position functor") {
    sim.setZmpPosition(calculateZmpExample);
    Vec3d pz = calculateZmpExample(p, v, 0);
    CHECK(sim.getZmpPosition(p, v, 0) == pz);
  }
}

Vec3d calculateReactForceExample(const Vec3d& p, const Vec3d& v,
                                 const double /* t */) {
  double k1 = 10;
  double k2 = 0.1;
  return k1 * p + k2 * v;
}
TEST_CASE("com_zmp_model_simulator: set reaction force functor",
          "[ComZmpModel][ComZmpModelSimulator]") {
  double mass = Random<double>(0, 2).get();
  auto model = ComZmpModelBuilder().setMass(mass).build();
  ComZmpModelSimulator sim(model);
  Vec3d p = Vec3d::Random();
  Vec3d v = Vec3d::Random();

  SECTION("default reaction force") {
    CHECK(sim.getReactForce(p, v, 0) == Vec3d(0, 0, mass * kGravAccel));
  }
  SECTION("set constant reaction force") {
    Vec3d f = Vec3d::Random();
    sim.setReactForce(f);
    CHECK(sim.getReactForce(p, v, 0) == f);
  }
  SECTION("set reaction force functor") {
    sim.setReactForce(calculateReactForceExample);
    Vec3d f = calculateReactForceExample(p, v, 0);
    CHECK(sim.getReactForce(p, v, 0) == f);
  }
}

Vec3d calculateExtForceExample(const Vec3d& p, const Vec3d& v, const double t) {
  if (t > 0.4 && t < 0.6)
    return p + v;
  else
    return kVec3dZero;
}
TEST_CASE("com_zmp_model_simulator: set external force functor",
          "[ComZmpModel][ComZmpModelSimulator]") {
  auto model = ComZmpModelBuilder().build();
  ComZmpModelSimulator sim(model);
  Vec3d p = Vec3d::Random();
  Vec3d v = Vec3d::Random();

  SECTION("default external force") {
    CHECK(sim.getExtForce(p, v, 0) == kVec3dZero);
  }
  SECTION("set constant external force") {
    Vec3d ef = Vec3d::Random();
    sim.setExtForce(ef);
    CHECK(sim.getExtForce(p, v, 0) == ef);
  }
  SECTION("set external force functor") {
    sim.setExtForce(calculateExtForceExample);
    CHECK(sim.getExtForce(p, v, 0) == kVec3dZero);
    CHECK(sim.getExtForce(p, v, 0.5) == p + v);
    CHECK(sim.getExtForce(p, v, 1) == kVec3dZero);
  }
}

TEST_CASE("com_zmp_model_simulator: clear external force",
          "[ComZmpModel][ComZmpModelSimulator]") {
  ComZmpModelSimulator sim;
  Vec3d ef = Vec3d::Random();
  sim.setExtForce(ef);
  REQUIRE(sim.getExtForce(kVec3dZero, kVec3dZero, 0) != kVec3dZero);
  sim.clearExtForce();
  CHECK(sim.getExtForce(kVec3dZero, kVec3dZero, 0) == kVec3dZero);
}

SCENARIO(
    "com_zmp_model_simulator: compute COM accel. when ZMP is treated as input",
    "[ComZmpModel][ComZmpModelSimulator]") {
  GIVEN("simulator in which ZMP is set as input") {
    auto model = getRandomModel();
    double mass = model.mass();
    Vec3d p = model.com_position();
    Vec3d v = model.com_velocity();
    ComZmpModelSimulator sim(model);
    sim.setZmpPosAsInput();

    WHEN("ZMP is not given") {
      THEN("COM accel. should be zero") {
        CHECK_THAT(sim.getComAccel(p, v, 0), ApproxEquals(kVec3dZero, kTOL));
      }
    }

    WHEN("ZMP given, but not vertical reaction force") {
      sim.setZmpPosition(calculateZmpExample);
      THEN("COM accel. along z-axis should be zero") {
        Vec3d f = Vec3d(0, 0, mass * kGravAccel);
        Vec3d pz = calculateZmpExample(p, v, 0);
        Vec3d expected_acc = cz::comAccel(p, pz, f, mass);
        REQUIRE(sim.getComAccel(p, v, 0).z() == Approx(0.0).margin(kTOL));
        CHECK_THAT(sim.getComAccel(p, v, 0), ApproxEquals(expected_acc, kTOL));
      }
    }

    WHEN("ZMP and vertical reaction force are given") {
      sim.setReactForce(calculateReactForceExample);
      sim.setZmpPosition(calculateZmpExample);
      THEN("COM accel. should be generated") {
        Vec3d f = calculateReactForceExample(p, v, 0);
        Vec3d pz = calculateZmpExample(p, v, 0);
        Vec3d expected_acc = cz::comAccel(p, pz, f, mass);
        REQUIRE(sim.getComAccel(p, v, 0).z() > -kGravAccel);
        CHECK_THAT(sim.getComAccel(p, v, 0), ApproxEquals(expected_acc, kTOL));
      }
    }

    WHEN("Additionally, external force is given") {
      sim.setReactForce(calculateReactForceExample);
      sim.setZmpPosition(calculateZmpExample);
      sim.setExtForce(calculateExtForceExample);
      THEN("COM accel. should be generated") {
        Vec3d f = calculateReactForceExample(p, v, 0.5);
        Vec3d ef = calculateExtForceExample(p, v, 0.5);
        Vec3d pz = calculateZmpExample(p, v, 0.5);
        Vec3d expected_acc = cz::comAccel(p, pz, f, mass);
        REQUIRE_THAT(sim.getComAccel(p, v, 0.5), !ApproxEquals(expected_acc));
        CHECK_THAT(sim.getComAccel(p, v, 0.5),
                   ApproxEquals(expected_acc + ef / mass, kTOL));
      }
    }
  }
}

SCENARIO(
    "com_zmp_model_simulator: compute COM accel. when react force is given",
    "[ComZmpModel][ComZmpModelSimulator]") {
  GIVEN("simulator in which reaction force is set as input") {
    auto model = getRandomModel();
    double mass = model.mass();
    Vec3d p = model.com_position();
    Vec3d v = model.com_velocity();
    ComZmpModelSimulator sim(model);
    sim.setReactForceAsInput();

    WHEN("Nothing is given") {
      THEN("COM accel. should be zero") {
        CHECK_THAT(sim.getComAccel(p, v, 0), ApproxEquals(kVec3dZero, kTOL));
      }
    }

    WHEN("reaction force is given") {
      sim.setReactForce(calculateReactForceExample);
      THEN("COM accel. along z-axis should be generated") {
        Vec3d f = calculateReactForceExample(p, v, 0);
        Vec3d expected_acc = cz::comAccel(f, mass);
        REQUIRE(sim.getComAccel(p, v, 0).z() > -kGravAccel);
        CHECK_THAT(sim.getComAccel(p, v, 0), ApproxEquals(expected_acc, kTOL));
      }
    }

    WHEN("ZMP is also given") {
      sim.setReactForce(calculateReactForceExample);
      sim.setZmpPosition(calculateZmpExample);
      THEN("ZMP should be ignored") {
        Vec3d f = calculateReactForceExample(p, v, 0);
        Vec3d pz = calculateZmpExample(p, v, 0);
        Vec3d wrong_acc = cz::comAccel(p, pz, f, mass);
        Vec3d expected_acc = cz::comAccel(f, mass);
        REQUIRE(sim.getComAccel(p, v, 0).z() > -kGravAccel);
        REQUIRE_THAT(sim.getComAccel(p, v, 0), !ApproxEquals(wrong_acc));
        CHECK_THAT(sim.getComAccel(p, v, 0), ApproxEquals(expected_acc, kTOL));
      }
    }

    WHEN("Additionally, external force is given") {
      sim.setReactForce(calculateReactForceExample);
      sim.setExtForce(calculateExtForceExample);
      THEN("COM accel. should be generated") {
        Vec3d f = calculateReactForceExample(p, v, 0.5);
        Vec3d ef = calculateExtForceExample(p, v, 0.5);
        Vec3d expected_acc = cz::comAccel(f, mass);
        REQUIRE_THAT(sim.getComAccel(p, v, 0.5), !ApproxEquals(expected_acc));
        CHECK_THAT(sim.getComAccel(p, v, 0.5),
                   ApproxEquals(expected_acc + ef / mass, kTOL));
      }
    }
  }
}

TEST_CASE(
    "com_zmp_model_simulator: check consistency between ZMP and react force",
    "[ComZmpModel][ComZmpModelSimulator]") {
  // setup
  auto model = getRandomModel();
  ComZmpModelSimulator sim(model);
  sim.setZmpPosAsInput();
  // let COM move left-forward and upward by setting ZMP and fz
  Vec3d p0 = model.com_position();
  double input_fz = (0.01 + model.mass()) * kGravAccel;
  Vec3d input_zmp(p0.x() - 0.1, p0.y() - 0.1, 0);
  sim.setReactForce(Vec3d(0, 0, input_fz));
  sim.setZmpPosition(input_zmp);
  // calculate expected ZMP, reaction force and COM accel.
  Vec3d expected_zmp = input_zmp;
  Vec3d expected_f = cz::reactForce(p0, input_zmp, input_fz);
  Vec3d expected_acc =
      cz::comAccel(p0, input_zmp, Vec3d(0, 0, input_fz), model.mass());
  // execute update
  sim.update();
  // check
  CHECK_THAT(sim.model().zmp_position(), ApproxEquals(expected_zmp, kTOL));
  CHECK_THAT(sim.model().reaction_force(), ApproxEquals(expected_f, kTOL));
  CHECK_THAT(sim.model().com_acceleration(), ApproxEquals(expected_acc, kTOL));
}

TEST_CASE(
    "com_zmp_model_simulator: check consistency between accel. and total force",
    "[ComZmpModel][ComZmpModelSimulator]") {
  // setup
  auto model = getRandomModel();
  ComZmpModelSimulator sim(model);
  sim.setZmpPosAsInput();
  // let COM move left-forward and upward by setting ZMP and fz
  Vec3d p0 = model.com_position();
  double input_fz = (0.01 + model.mass()) * kGravAccel;
  Vec3d input_zmp(p0.x() - 0.1, p0.y() - 0.1, 0);
  sim.setReactForce(Vec3d(0, 0, input_fz));
  sim.setZmpPosition(input_zmp);
  // additionally, external force is applied
  Vec3d ef = Random<Vec3d>(-0.01, 0.01).get();
  sim.setExtForce(ef);
  // calculate expected ZMP, reaction/total force and COM accel.
  Vec3d expected_zmp = input_zmp;
  Vec3d expected_rf = cz::reactForce(p0, input_zmp, input_fz);
  Vec3d expected_ef = ef;
  Vec3d expected_tf = expected_rf + expected_ef;
  Vec3d expected_acc =
      cz::comAccel(p0, input_zmp, Vec3d(0, 0, input_fz), model.mass(), ef);
  // execute update
  sim.update();
  // check
  CHECK_THAT(sim.model().zmp_position(), ApproxEquals(expected_zmp, kTOL));
  CHECK_THAT(sim.model().reaction_force(), ApproxEquals(expected_rf, kTOL));
  CHECK_THAT(sim.model().external_force(), ApproxEquals(expected_ef, kTOL));
  CHECK_THAT(sim.model().total_force(), ApproxEquals(expected_tf, kTOL));
  CHECK_THAT(sim.model().com_acceleration(), ApproxEquals(expected_acc, kTOL));
}

SCENARIO("com_zmp_model_simulator: check if COM move after update",
         "[ComZmpModel][ComZmpModelSimulator]") {
  // prepare COM-ZMP model in which COM resides at (0, 0, 1)
  auto model = ComZmpModelBuilder().build();
  ComZmpModelSimulator sim(model);
  sim.setZmpPosAsInput();

  GIVEN("let COM move left-forward and upward") {
    const Vec3d input_zmp(-0.1, -0.1, 0);
    const double input_fz = 1.01 * kGravAccel;
    sim.setReactForce(Vec3d(0, 0, input_fz));
    sim.setZmpPosition(input_zmp);
    REQUIRE_THAT(sim.model().com_position(), ApproxEquals(kVec3dZ, kTOL));
    REQUIRE_THAT(sim.model().com_velocity(), ApproxEquals(kVec3dZero, kTOL));

    WHEN("update once") {
      sim.update();

      THEN("COM velocity should be modified towards left-forward and upward") {
        const Vec3d v = sim.model().com_velocity();
        CHECK(v.x() > 0.0);
        CHECK(v.y() > 0.0);
        CHECK(v.z() > 0.0);
      }
      THEN("but COM should still remain at the initial position") {
        const Vec3d p = sim.model().com_position();
        CHECK(p.x() == Approx(0.0).margin(1e-5));
        CHECK(p.y() == Approx(0.0).margin(1e-5));
        CHECK(p.z() == Approx(1.0).margin(1e-5));
      }
    }
    WHEN("update once more") {
      sim.update();

      THEN("COM velocity should be left-forward and upward") {
        const Vec3d v = sim.model().com_velocity();
        CHECK(v.x() > 0.0);
        CHECK(v.y() > 0.0);
        CHECK(v.z() > 0.0);
      }
      THEN("and COM should also move left-forward and upward") {
        const Vec3d p = sim.model().com_position();
        CHECK(p.x() > 0.0);
        CHECK(p.y() > 0.0);
        CHECK(p.z() > 1.0);
      }
    }
  }
}

TEST_CASE("com_zmp_model_simulator: ", "[ComZmpModel][ComZmpModelSimulator]") {}

}  // namespace
}  // namespace holon
