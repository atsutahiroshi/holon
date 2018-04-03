/* com_zmp_model - COM-ZMP model
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

#include "holon/corelib/humanoid/com_zmp_model.hpp"

#include <zm/zm_ieee.h>
#include <roki/rk_g.h>
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

#include "catch.hpp"
#include "holon/test/util/catch/custom_matchers.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {

namespace experimental {

namespace {

using Catch::Matchers::Equals;
using com_zmp_model_formula::computeSqrZeta;
using com_zmp_model_formula::computeZeta;
using com_zmp_model_formula::computeReactForce;
using com_zmp_model_formula::computeComAcc;

const double G = RK_G;

void RandomizeData(ComZmpModelData data) {
  Fuzzer fuzz;
  Fuzzer positive(0, 10);

  data.get().mass = positive();
  data.get().nu = fuzz.get<Vec3D>();
  data.get().com_position = fuzz.get<Vec3D>();
  data.get().com_position.set_z(positive());
  data.get().com_velocity = fuzz.get<Vec3D>();
  data.get().com_acceleration = fuzz.get<Vec3D>();
  data.get().zmp_position = fuzz.get<Vec3D>();
  data.get().reaction_force = fuzz.get<Vec3D>();
  data.get().external_force = fuzz.get<Vec3D>();
  data.get().total_force = fuzz.get<Vec3D>();
}

#define CHECK_COMZMPMODELDATA_MEMBER(a, b, var) \
  CHECK(a.get<0>().var == b.get<0>().var)
void CheckData(const ComZmpModelData& a, const ComZmpModelData& b) {
  CHECK_COMZMPMODELDATA_MEMBER(a, b, mass);
  CHECK_COMZMPMODELDATA_MEMBER(a, b, nu);
  CHECK_COMZMPMODELDATA_MEMBER(a, b, com_position);
  CHECK_COMZMPMODELDATA_MEMBER(a, b, com_velocity);
  CHECK_COMZMPMODELDATA_MEMBER(a, b, com_acceleration);
  CHECK_COMZMPMODELDATA_MEMBER(a, b, zmp_position);
  CHECK_COMZMPMODELDATA_MEMBER(a, b, reaction_force);
  CHECK_COMZMPMODELDATA_MEMBER(a, b, external_force);
  CHECK_COMZMPMODELDATA_MEMBER(a, b, total_force);
}

// ComZmpModel class
TEST_CASE("Check C'tors of ComZmpModel", "[ComZmpModel][ctor]") {
  double default_mass = 1;
  Vec3D default_com_position = {0, 0, 1};

  SECTION("default constructor (no parameters)") {
    ComZmpModel model;
    CHECK(model.states().mass == default_mass);
    CHECK(model.states().com_position == default_com_position);
    CHECK(model.initial_com_position() == default_com_position);
    CHECK(model.time() == 0.0);
  }

  SECTION("ComZmpModel(double t_mass)") {
    Fuzzer fuzz(0, 10);
    for (auto i = 0; i < 10; ++i) {
      double m = fuzz.get();
      auto p0 = fuzz.get<Vec3D>();
      ComZmpModel model(p0, m);
      CHECK(model.states().mass == m);
      CHECK(model.states().com_position == p0);
      CHECK(model.initial_com_position() == p0);
      CHECK(model.time() == 0.0);
    }

    SECTION("mass should be positive") {
      zEchoOff();
      ComZmpModel model1(kVec3DZ, -1.0);
      CHECK(model1.states().mass == 1.0);
      ComZmpModel model2(kVec3DZ, 0.0);
      CHECK(model2.states().mass == 1.0);
      zEchoOn();
    }
  }

  SECTION("ComZmpModel(Data t_data)") {
    auto data = make_data<ComZmpModelData>();
    data.get().com_position = Vec3D(0.1, 0.2, 0.3);
    ComZmpModel model(data);
    CHECK(model.data() == data);
    CHECK(model.initial_com_position() == data.get().com_position);
    CHECK(model.time() == 0.0);
  }
}

TEST_CASE("Check accessor/mutator of ComZmpModel",
          "[ComZmpModel][accessor][mutator]") {
  Fuzzer fuzz;
  ComZmpModel model;

  SECTION("data") {
    auto data1 = make_data<ComZmpModelData>();
    REQUIRE(model.data() != data1);
    model.set_data(data1);
    CHECK(model.data() == data1);

    auto data2 = make_data<ComZmpModelData>();
    REQUIRE(model.data() != data2);
    model.set_data(data2);
    CHECK(model.data() == data2);
  }
  SECTION("initial COM position") {
    auto v = fuzz.get<Vec3D>();
    REQUIRE(model.initial_com_position() != v);
    model.set_initial_com_position(v);
    CHECK(model.initial_com_position() == v);
  }
  SECTION("time step") {
    Fuzzer fuzz(0.0001, 0.1);
    double dt = fuzz.get();
    model.set_time_step(dt);
    CHECK(model.time_step() == Approx(dt));
  }
  SECTION("non-positive time step is not allowed") {
    zEchoOff();
    model.set_time_step(0.1);
    CHECK(model.time_step() != 0.001);
    model.set_time_step(0);
    CHECK(model.time_step() == 0.001);

    model.set_time_step(0.1);
    CHECK(model.time_step() != 0.001);
    model.set_time_step(-0.01);
    CHECK(model.time_step() == 0.001);
    zEchoOn();
  }
}

TEST_CASE("ComZmpModel::mass() returns mass value", "[ComZmpModel][mass]") {
  SECTION("case 1") {
    ComZmpModel model;
    auto data = model.data();
    CHECK(model.mass() == 1.0);
    data.get().mass = 10;
    CHECK(model.mass() == 10.0);
  }
  SECTION("case 2") {
    auto data = make_data<ComZmpModelData>(kVec3DZ, 2);
    ComZmpModel model(data);
    CHECK(model.mass() == 2.0);
    data.get().mass = 20;
    CHECK(model.mass() == 20.0);
  }
  SECTION("case 3") {
    ComZmpModel model;
    auto data = make_data<ComZmpModelData>(kVec3DZ, 3);
    model.set_data(data);
    CHECK(model.mass() == 3.0);
    data.get().mass = 30;
    CHECK(model.mass() == 30.0);
  }
}

SCENARIO("Check reset in ComZmpModel", "[ComZmpModel][reset]") {
  GIVEN("initialize with random values") {
    auto data = make_data<ComZmpModelData>();
    ComZmpModel model(data);
    Fuzzer fuzz;

    data.get().com_position = fuzz.get<Vec3D>();
    data.get().com_velocity = fuzz.get<Vec3D>();
    REQUIRE(model.states().com_position != Vec3D(0, 0, 1));
    REQUIRE(model.states().com_velocity != Vec3D(0, 0, 0));

    WHEN("reset COM position") {
      Vec3D p = fuzz.get<Vec3D>();
      model.reset(p);

      THEN("COM position should be that value and velocity should be zero") {
        CHECK(model.states().com_position == p);
        CHECK(model.initial_com_position() == p);
        CHECK(model.states().com_velocity == kVec3DZero);
      }
    }
  }
}

TEST_CASE("Check if reset in ComZmpModel resets time to zero",
          "[ComZmpModel][reset]") {
  ComZmpModel model;
  model.update();
  model.update();
  REQUIRE(model.time() != 0);
  model.reset();
  CHECK(model.time() == 0.0);
}

TEST_CASE("Check copy_data in ComZmpModel", "[ComZmpModel][copy_data]") {
  Fuzzer fuzz;

  SECTION("ComZmpModel::copy_data(const ComZmpModel&)") {
    ComZmpModel a, b;
    RandomizeData(a.data());
    b.copy_data(a);
    CheckData(b.data(), a.data());
    // initial COM position should be intact
    CHECK(b.initial_com_position() != a.states().com_position);
  }
  SECTION("ComZmpModel::copy_data(const Data&)") {
    ComZmpModelData data;
    RandomizeData(data);

    ComZmpModel model;
    model.copy_data(data);
    CheckData(model.data(), data);
    // initial COM position should be intact
    CHECK(model.initial_com_position() != data.get().com_position);
  }
}

TEST_CASE("ComZmpModel::setZmpPosition sets fixed ZMP position",
          "[ComZmpModel][setZmpPosition]") {
  ComZmpModel model;
  Fuzzer fuzz;
  auto p = fuzz.get<Vec3D>();
  auto v = fuzz.get<Vec3D>();
  auto t = fuzz();
  SECTION("set ZMP only") {
    auto pz = fuzz.get<Vec3D>();
    model.setZmpPosition(pz);
    CHECK(model.system().zmp_position(p, v, t) == pz);
  }
  SECTION("ZMP and fz") {
    auto pz = fuzz.get<Vec3D>();
    auto fz = fuzz();
    model.setZmpPosition(pz, fz);
    CHECK(model.system().zmp_position(p, v, t) == pz);
    CHECK(model.system().reaction_force(p, v, t) == Vec3D(0, 0, fz));
  }
}

TEST_CASE("ComZmpModel::removeZmpPosition removes ZMP position",
          "[ComZmpModel][removeZmpPosition]") {
  ComZmpModel model;
  Fuzzer fuzz;
  auto p = fuzz.get<Vec3D>();
  auto v = fuzz.get<Vec3D>();
  auto t = fuzz();
  SECTION("set ZMP only then remove it") {
    auto pz = fuzz.get<Vec3D>();
    model.setZmpPosition(pz);
    CHECK(model.system().zmp_position(p, v, t) == pz);
    model.removeZmpPosition();
    CHECK_FALSE(model.system().isZmpPositionSet());
  }
  SECTION("set ZMP and fz then remove them") {
    auto pz = fuzz.get<Vec3D>();
    auto fz = fuzz();
    model.setZmpPosition(pz, fz);
    CHECK(model.system().zmp_position(p, v, t) == pz);
    CHECK(model.system().reaction_force(p, v, t) == Vec3D(0, 0, fz));
    model.removeZmpPosition();
    CHECK_FALSE(model.system().isZmpPositionSet());
    CHECK(model.system().reaction_force(p, v, t) == Vec3D(0, 0, G));
  }
}

TEST_CASE("ComZmpModel::setReactionForce sets reaction force",
          "[ComZmpModel][setReactionForce]") {
  ComZmpModel model;
  Fuzzer fuzz;
  auto p = fuzz.get<Vec3D>();
  auto v = fuzz.get<Vec3D>();
  auto t = fuzz();
  auto f = fuzz.get<Vec3D>();
  model.setReactionForce(f);
  CHECK(model.system().reaction_force(p, v, t) == f);
}

TEST_CASE("ComZmpModel::removeReactionForce removess reaction force",
          "[ComZmpModel][removeReactionForce]") {
  ComZmpModel model;
  Fuzzer fuzz;
  auto p = fuzz.get<Vec3D>();
  auto v = fuzz.get<Vec3D>();
  auto t = fuzz();
  auto f = fuzz.get<Vec3D>();
  model.setReactionForce(f);
  CHECK(model.system().reaction_force(p, v, t) == f);
  model.removeReactionForce();
  CHECK(model.system().reaction_force(p, v, t) == Vec3D(0, 0, G));
}

TEST_CASE("ComZmpModel::setExternalForce sets external force",
          "[ComZmpModel][setExternalForce]") {
  ComZmpModel model;
  Fuzzer fuzz;
  auto p = fuzz.get<Vec3D>();
  auto v = fuzz.get<Vec3D>();
  auto t = fuzz();
  auto fe = fuzz.get<Vec3D>();
  model.setExternalForce(fe);
  CHECK(model.system().external_force(p, v, t) == fe);
}

TEST_CASE("ComZmpModel::removeExternalForce removes external force",
          "[ComZmpModel][removeExternalForce]") {
  ComZmpModel model;
  Fuzzer fuzz;
  auto p = fuzz.get<Vec3D>();
  auto v = fuzz.get<Vec3D>();
  auto t = fuzz();
  auto fe = fuzz.get<Vec3D>();
  model.setExternalForce(fe);
  CHECK(model.system().external_force(p, v, t) == fe);
  model.removeExternalForce();
  CHECK(model.system().external_force(p, v, t) == kVec3DZero);
}

TEST_CASE("ComZmpModel::update counts time", "[ComZmpModel][update]") {
  ComZmpModel model;
  REQUIRE(model.time() == 0.0);
  model.update();
  REQUIRE(model.time() == Approx(model.time_step()));
  model.update();
  REQUIRE(model.time() == Approx(2. * model.time_step()));
  model.update();
  REQUIRE(model.time() == Approx(3. * model.time_step()));
}

TEST_CASE("ComZmpModel::update(double) modify step time",
          "[ComZmpModel][update]") {
  ComZmpModel model;
  Fuzzer fuzz(0.0001, 0.1);
  double dt1 = fuzz.get();
  double dt2 = fuzz.get();

  REQUIRE(model.time_step() != dt1);
  model.update(dt1);
  REQUIRE(model.time_step() == dt1);
  model.update();
  REQUIRE(model.time_step() == dt1);

  REQUIRE(model.time_step() != dt2);
  model.update(dt2);
  REQUIRE(model.time_step() == dt2);
  model.update();
  REQUIRE(model.time_step() == dt2);
}

TEST_CASE("ComZmpModel::update(double) counts time correctly",
          "[ComZmpModel][update]") {
  ComZmpModel model;
  Fuzzer fuzz(0.0001, 0.1);
  double dt1 = fuzz.get();
  double dt2 = fuzz.get();
  double t = 0;

  REQUIRE(model.time() == Approx(t));
  model.update(dt1);
  t += dt1;
  REQUIRE(model.time() == Approx(t));
  model.update();
  t += dt1;
  REQUIRE(model.time() == Approx(t));

  model.update(dt2);
  t += dt2;
  REQUIRE(model.time() == Approx(t));
  model.update();
  t += dt2;
  REQUIRE(model.time() == Approx(t));
}

TEST_CASE("ComZmpModel::update computes COM acceleration",
          "[ComZmpModel][update]") {
  ComZmpModel model;

  SECTION("update acceleration") {
    struct testcase_t {
      Vec3D com_pos;
      Vec3D com_vel;
      Vec3D zmp_pos;
    } testcases[] = {{{0, 0, 1}, {0, 0, 0}, {1, 0, 0}},
                     {{0, 0.1, 1}, {0.1, -0.1, 0}, {0.2, 0.1, 0}}};
    for (auto& c : testcases) {
      Vec3D f = computeReactForce(c.com_pos, c.zmp_pos, model.mass() * G);
      Vec3D expected_com_acc = computeComAcc(f, model.mass());

      model.states().com_position = c.com_pos;
      model.states().com_velocity = c.com_vel;
      // model.inputZmpPos(c.zmp_pos);
      model.setZmpPosition(c.zmp_pos);
      model.update();
      CHECK(model.states().com_acceleration == expected_com_acc);
    }
  }
}

SCENARIO("update COM position, velocity, acceleration", "[corelib][humanoid]") {
  GIVEN("COM stays at (0, 0, 1)") {
    auto data = make_data<ComZmpModelData>();
    ComZmpModel model(data);

    WHEN("input ZMP position as (-1, -0.5, 0) and update") {
      Vec3D zmp_pos = {-1, -0.5, 0};
      // model.inputZmpPos(zmp_pos);
      model.setZmpPosition(zmp_pos);
      REQUIRE(model.states().com_position == kVec3DZ);
      REQUIRE(model.states().com_velocity == kVec3DZero);
      REQUIRE(model.update());

      THEN("horizontal velocity should be positive") {
        Vec3D vel = model.states().com_velocity;
        CHECK(vel.x() > 0.0);
        CHECK(vel.y() > 0.0);
      }
      THEN("horizontal position should still be at zero") {
        Vec3D pos = model.states().com_position;
        CHECK(pos.x() == Approx(0.0).margin(1e-5));
        CHECK(pos.y() == Approx(0.0).margin(1e-5));
      }

      WHEN("and update once more") {
        data.get().zmp_position = zmp_pos;
        REQUIRE(model.update());

        THEN("horizontal velocity should be positive") {
          Vec3D vel = model.states().com_velocity;
          CHECK(vel.x() > 0.0);
          CHECK(vel.y() > 0.0);
        }
        THEN("horizontal position should move forward") {
          Vec3D pos = model.states().com_position;
          CHECK(pos.x() > 0.0);
          CHECK(pos.y() > 0.0);
        }
      }
    }
  }
}

TEST_CASE("when COM height is zero, update should fail") {
  auto data = make_data<ComZmpModelData>();
  ComZmpModel model(data);
  Vec3D p = {0, 0, 0};

  data.get().com_position = p;
  model.setZmpPosition(kVec3DZero);
  zEchoOff();
  CAPTURE(model.states().com_position);
  CAPTURE(model.states().com_velocity);
  CAPTURE(model.states().com_acceleration);
  CHECK_FALSE(model.update());
  zEchoOn();
}

TEST_CASE("ComZmpModel::update keeps consistency of vertical reaction force",
          "[ComZmpModel][update]") {
  ComZmpModel model;
  auto data = model.data();
  model.reset(Vec3D(0, 0, 0.42));

  double desired_fz = 10;
  Vec3D desired_zmp = {-1, 0.42, 0};
  model.setZmpPosition(desired_zmp, desired_fz);
  model.update();
  CHECK(data.get().reaction_force == Vec3D(10. / 0.42, -10, desired_fz));
  CHECK(data.get().com_acceleration == Vec3D(10. / 0.42, -10, 10 - G));
}

TEST_CASE("ComZmpModel::update computes total force being applied to COM",
          "[ComZmpModel][update]") {
  ComZmpModel model;
  auto data = model.data();
  model.reset(Vec3D(0, 0, 0.42));

  double desired_fz = 15;
  Vec3D desired_zmp = {-1, 0.42, 0};
  Vec3D ext_force = {1.2, -1.2, -0.2};
  model.setZmpPosition(desired_zmp, desired_fz);
  model.setExternalForce(ext_force);
  model.update();
  CHECK(data.get().reaction_force == Vec3D(15. / 0.42, -15, desired_fz));
  CHECK(data.get().total_force == Vec3D(15. / 0.42 + 1.2, -16.2, 14.8));
  CHECK(data.get().com_acceleration ==
        Vec3D(15. / 0.42 + 1.2, -16.2, 14.8 - G));
}

TEST_CASE("ComZmpModel::update() updates external force in data",
          "[ComZmpModel][update]") {
  ComZmpModel model;
  auto data = model.data();
  model.reset(Vec3D(0, 0, 0.42));

  Vec3D ext_force = {1.5, -1.5, -10};
  model.setExternalForce(ext_force);
  REQUIRE(data.get().reaction_force == Vec3D(0, 0, G));
  model.update();
  CHECK(data.get().total_force == Vec3D(1.5, -1.5, G - 10));
  CHECK(data.get().com_acceleration == Vec3D(1.5, -1.5, -10));
  CHECK(data.get().external_force == ext_force);
}

}  // namespace

}  // namespace experimental

}  // namespace holon
