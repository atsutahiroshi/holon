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
  SECTION("method 1") {
    sim.setInputType(Type::kZmpPosition);
    CHECK(sim.getInputType() == Type::kZmpPosition);
  }
  SECTION("method 2") {
    sim.setZmpPosAsInput();
    CHECK(sim.getInputType() == Type::kZmpPosition);
  }
}

TEST_CASE("com_zmp_model_simulator: set reaction force as input",
          "[ComZmpModel][ComZmpModelSimulator]") {
  ComZmpModelSimulator sim;
  using Type = ComZmpModelSimulator::InputType;
  REQUIRE(sim.getInputType() == Type::kNotDetermined);
  SECTION("method 1") {
    sim.setInputType(Type::kReactionForce);
    CHECK(sim.getInputType() == Type::kReactionForce);
  }
  SECTION("method 2") {
    sim.setReactForceAsInput();
    CHECK(sim.getInputType() == Type::kReactionForce);
  }
}

Vec3d calculateZmpExample(const Vec3d& p, const Vec3d& v,
                          const double /* t */) {
  double k1 = 10;
  double k2 = 0.1;
  return p + k1 * p + k2 * v;
}
TEST_CASE("com_zmp_model_simulator: set ZMP position functor",
          "[ComZmpModel][ComZmpModelSimulator]") {
  double vhp = Random<double>(0, 1).get();
  Vec3d p0 = Random<Vec3d>(-1, 1).get();
  p0[2] += 2;  // make COM height be larger than vhp
  auto model = ComZmpModelBuilder()
                   .setVirtualHorizontalPlane(vhp)
                   .setComPosition(p0)
                   .build();
  ComZmpModelSimulator sim(model);
  Vec3d p = Vec3d::Random();
  Vec3d v = Vec3d::Random();

  SECTION("default ZMP position") {
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
  Vec3d f = Vec3d::Zero();
  f.z() = k1 * p.z() + k2 * v.z();
  return f;
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
  if (t > 0.5 && t < 0.7)
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
    CHECK(sim.getExtForce(p, v, 0.6) == p + v);
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

TEST_CASE("com_zmp_model_simulator: ", "[ComZmpModel][ComZmpModelSimulator]") {}

}  // namespace
}  // namespace holon
