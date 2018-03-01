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
namespace {

using Catch::Matchers::Equals;
using ComZmpModelFormula::computeSqrZeta;
using ComZmpModelFormula::computeZeta;
using ComZmpModelFormula::computeReactForce;
using ComZmpModelFormula::computeComAcc;

const double G = RK_G;

void RandomizeData(ComZmpModelData* data) {
  Fuzzer fuzz;
  Fuzzer positive(0, 10);

  data->mass = positive();
  data->nu = fuzz.get<Vec3D>();
  data->com_position = fuzz.get<Vec3D>();
  data->com_position.set_z(positive());
  data->com_velocity = fuzz.get<Vec3D>();
  data->com_acceleration = fuzz.get<Vec3D>();
  data->zmp_position = fuzz.get<Vec3D>();
  data->reaction_force = fuzz.get<Vec3D>();
  data->external_force = fuzz.get<Vec3D>();
  data->total_force = fuzz.get<Vec3D>();
}

#define CHECK_COMZMPMODELDATA_MEMBER(a, b, var) CHECK(a.var == b.var)
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
TEST_CASE("ComZmpModel constructor", "[corelib][humanoid][ComZmpModel]") {
  double default_mass = 1;
  Vec3D default_com_position = {0, 0, 1};

  SECTION("default constructor (no parameters)") {
    ComZmpModel model;
    CHECK(model.data().mass == default_mass);
    CHECK(model.data().com_position == default_com_position);
    CHECK(model.initial_com_position() == default_com_position);
    CHECK(model.time() == 0.0);
  }

  SECTION("ComZmpModel(double t_mass)") {
    Fuzzer fuzz(0, 10);
    for (auto i = 0; i < 10; ++i) {
      double m = fuzz.get();
      auto p0 = fuzz.get<Vec3D>();
      ComZmpModel model(p0, m);
      CHECK(model.data().mass == m);
      CHECK(model.data().com_position == p0);
      CHECK(model.initial_com_position() == p0);
      CHECK(model.time() == 0.0);
    }

    SECTION("mass should be positive") {
      zEchoOff();
      ComZmpModel model1(kVec3DZ, -1.0);
      CHECK(model1.data().mass == 1.0);
      ComZmpModel model2(kVec3DZ, 0.0);
      CHECK(model2.data().mass == 1.0);
      zEchoOn();
    }
  }

  SECTION("ComZmpModel(Data t_data)") {
    auto data = createComZmpModelData();
    data->com_position = Vec3D(0.1, 0.2, 0.3);
    ComZmpModel model(data);
    CHECK(&model.data() == data.get());
    CHECK(model.initial_com_position() == data->com_position);
    CHECK(model.time() == 0.0);
  }
}

TEST_CASE("ComZmpModel: accessor/mutator of data",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("ComZmpModel can hold another Data pointer") {
    auto data1 = createComZmpModelData();
    REQUIRE(&model.data() != data1.get());
    model.set_data_ptr(data1);
    CHECK(&model.data() == data1.get());

    auto data2 = createComZmpModelData();
    REQUIRE(&model.data() != data2.get());
    model.set_data_ptr(data2);
    CHECK(&model.data() == data2.get());
  }
}

TEST_CASE("ComZmpModel: accessor/mutator", "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;
  Fuzzer fuzz;

  SECTION("initial COM position") {
    auto v = fuzz.get<Vec3D>();
    REQUIRE(model.initial_com_position() != v);
    model.set_initial_com_position(v);
    CHECK(model.initial_com_position() == v);
  }
}

TEST_CASE("ComZmpModel: accessor/mutator of time step",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("default value is 0.001") {
    CHECK(model.time_step() == Approx(0.001));
  }
  SECTION("set a value") {
    Fuzzer fuzz(0.0001, 0.1);
    double dt = fuzz.get();
    model.set_time_step(dt);
    CHECK(model.time_step() == Approx(dt));
  }
  SECTION("non-positive values are not allowed") {
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

TEST_CASE("ComZmpModel::mass() returns mass value",
          "[corelib][humanoid][ComZmpModel]") {
  SECTION("case 1") {
    ComZmpModel model;
    auto data = model.data_ptr();
    CHECK(model.mass() == 1.0);
    data->mass = 10;
    CHECK(model.mass() == 10.0);
  }
  SECTION("case 2") {
    auto data = createComZmpModelData(kVec3DZ, 2);
    ComZmpModel model(data);
    CHECK(model.mass() == 2.0);
    data->mass = 20;
    CHECK(model.mass() == 20.0);
  }
  SECTION("case 3") {
    ComZmpModel model;
    auto data = createComZmpModelData(kVec3DZ, 3);
    model.set_data_ptr(data);
    CHECK(model.mass() == 3.0);
    data->mass = 30;
    CHECK(model.mass() == 30.0);
  }
}

SCENARIO("ComZmpModel: function to reset COM position",
         "[corelib][humanoid][ComZmpModel]") {
  GIVEN("initialize with random values") {
    auto data = createComZmpModelData();
    ComZmpModel model(data);
    Fuzzer fuzz;

    fuzz.randomize(data->com_position);
    fuzz.randomize(data->com_velocity);
    REQUIRE_THAT(model.data().com_position, !Equals(Vec3D(0, 0, 1)));
    REQUIRE_THAT(model.data().com_velocity, !Equals(Vec3D(0, 0, 0)));

    WHEN("reset COM position") {
      Vec3D p;
      fuzz.randomize(p);
      model.reset(p);

      THEN("COM position should be that value and velocity should be zero") {
        CHECK_THAT(model.data().com_position, Equals(p));
        CHECK_THAT(model.initial_com_position(), Equals(p));
        CHECK_THAT(model.data().com_velocity, Equals(kVec3DZero));
      }
    }
  }
}

TEST_CASE("ComZmpModel::reset() resets time to zero",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;
  model.update();
  model.update();
  REQUIRE(model.time() != 0);
  model.reset(kVec3DZ);
  CHECK(model.time() == 0.0);
}

TEST_CASE("ComZmpModel::copy_data should copy data from argument",
          "[corelib][humanoid][ComZmpModel]") {
  Fuzzer fuzz;

  SECTION("ComZmpModel::copy_data(const ComZmpModel&)") {
    ComZmpModel a, b;
    RandomizeData(a.data_ptr().get());
    b.copy_data(a);
    CheckData(b.data(), a.data());
    // initial COM position should be intact
    CHECK(b.initial_com_position() != a.data().com_position);
  }
  SECTION("ComZmpModel::copy_data(const Data&)") {
    ComZmpModelData data;
    RandomizeData(&data);

    ComZmpModel model;
    model.copy_data(data);
    CheckData(model.data(), data);
    // initial COM position should be intact
    CHECK(model.initial_com_position() != data.com_position);
  }
}

TEST_CASE("ComZmpModel::update counts time",
          "[corelib][humanoid][ComZmpModel]") {
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
          "[corelib][humanoid][ComZmpModel]") {
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
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;
  Fuzzer fuzz(0.0001, 0.1);
  double dt1 = fuzz.get();
  double dt2 = fuzz.get();
  double t = 0;

  // REQUIRE(model.time_step() != dt1);
  REQUIRE(model.time() == t);
  model.update(dt1);
  t += dt1;
  REQUIRE(model.time() == t);
  // REQUIRE(model.time_step() == dt1);
  model.update();
  t += dt1;
  REQUIRE(model.time() == t);
  // REQUIRE(model.time_step() == dt1);

  // REQUIRE(model.time_step() != dt2);
  model.update(dt2);
  t += dt2;
  REQUIRE(model.time() == t);
  // REQUIRE(model.time_step() == dt2);
  model.update();
  t += dt2;
  REQUIRE(model.time() == t);
  // REQUIRE(model.time_step() == dt2);
}

TEST_CASE("ComZmpModel::update computes COM acceleration",
          "[corelib][humanoid][ComZmpModel]") {
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

      model.data_ptr()->com_position = c.com_pos;
      model.data_ptr()->com_velocity = c.com_vel;
      // model.inputZmpPos(c.zmp_pos);
      model.setFixedZmpPosition(c.zmp_pos);
      model.update();
      CHECK_THAT(model.data().com_acceleration, Equals(expected_com_acc));
    }
  }
}

SCENARIO("update COM position, velocity, acceleration", "[corelib][humanoid]") {
  GIVEN("COM stays at (0, 0, 1)") {
    auto data = createComZmpModelData();
    ComZmpModel model(data);

    WHEN("input ZMP position as (-1, -0.5, 0) and update") {
      Vec3D zmp_pos = {-1, -0.5, 0};
      // model.inputZmpPos(zmp_pos);
      model.setFixedZmpPosition(zmp_pos);
      REQUIRE(model.data().com_position == kVec3DZ);
      REQUIRE(model.data().com_velocity == kVec3DZero);
      REQUIRE(model.update());

      THEN("horizontal velocity should be positive") {
        Vec3D vel = model.data().com_velocity;
        CHECK(vel.x() > 0.0);
        CHECK(vel.y() > 0.0);
      }
      THEN("horizontal position should still be at zero") {
        Vec3D pos = model.data().com_position;
        CHECK(pos.x() == Approx(0.0).margin(1e-5));
        CHECK(pos.y() == Approx(0.0).margin(1e-5));
      }

      WHEN("and update once more") {
        data->zmp_position = zmp_pos;
        REQUIRE(model.update());

        THEN("horizontal velocity should be positive") {
          Vec3D vel = model.data().com_velocity;
          CHECK(vel.x() > 0.0);
          CHECK(vel.y() > 0.0);
        }
        THEN("horizontal position should move forward") {
          Vec3D pos = model.data().com_position;
          CHECK(pos.x() > 0.0);
          CHECK(pos.y() > 0.0);
        }
      }
    }
  }
}

TEST_CASE("when COM height is zero, update should fail") {
  auto data = createComZmpModelData();
  ComZmpModel model(data);
  Vec3D p = {0, 0, 0};

  data->com_position = p;
  model.setFixedZmpPosition(kVec3DZero);
  zEchoOff();
  CAPTURE(model.data().com_position);
  CAPTURE(model.data().com_velocity);
  CAPTURE(model.data().com_acceleration);
  CHECK_FALSE(model.update());
  zEchoOn();
}

TEST_CASE("ComZmpModel::update keeps consistency of vertical reaction force",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;
  auto data = model.data_ptr();
  model.reset(Vec3D(0, 0, 0.42));

  double desired_fz = 10;
  Vec3D desired_zmp = {-1, 0.42, 0};
  // model.inputZmpPos(desired_zmp, desired_fz);
  model.setFixedZmpPosition(desired_zmp, desired_fz);
  model.update();
  CHECK(data->reaction_force == Vec3D(10. / 0.42, -10, desired_fz));
  CHECK(data->com_acceleration == Vec3D(10. / 0.42, -10, 10 - G));
}

TEST_CASE("ComZmpModel::update computes total force being applied to COM",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;
  auto data = model.data_ptr();
  model.reset(Vec3D(0, 0, 0.42));

  double desired_fz = 15;
  Vec3D desired_zmp = {-1, 0.42, 0};
  Vec3D ext_force = {1.2, -1.2, -0.2};
  model.setFixedZmpPosition(desired_zmp, desired_fz);
  model.setFixedExternalForce(ext_force);
  model.update();
  CHECK(data->reaction_force == Vec3D(15. / 0.42, -15, desired_fz));
  CHECK(data->total_force == Vec3D(15. / 0.42 + 1.2, -16.2, 14.8));
  CHECK(data->com_acceleration == Vec3D(15. / 0.42 + 1.2, -16.2, 14.8 - G));
}

TEST_CASE("ComZmpModel::update() updates external force in data",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;
  auto data = model.data_ptr();
  model.reset(Vec3D(0, 0, 0.42));

  Vec3D ext_force = {1.5, -1.5, -10};
  model.setFixedExternalForce(ext_force);
  REQUIRE(data->reaction_force == Vec3D(0, 0, G));
  model.update();
  CHECK(data->total_force == Vec3D(1.5, -1.5, G - 10));
  CHECK(data->com_acceleration == Vec3D(1.5, -1.5, -10));
  CHECK(data->external_force == ext_force);
}

}  // namespace
}  // namespace holon
