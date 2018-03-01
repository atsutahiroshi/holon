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
#include <memory>
#include <utility>

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
using ComZmpModelFormula::isMassValid;
using ComZmpModelFormula::isComZmpDiffValid;
using ComZmpModelFormula::isReactionForceValid;
using ComZmpModelFormula::isComAccelerationValid;

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

TEST_CASE("ComZmpModelData: constructor",
          "[corelib][humanoid][ComZmpModelData]") {
  double default_mass = 1;
  Vec3D default_com_position = {0, 0, 1};

  SECTION("default constructor (no parameters)") {
    ComZmpModelData data;
    CHECK(data.mass == default_mass);
    CHECK_THAT(data.nu, Equals(kVec3DZ));
    CHECK_THAT(data.com_position, Equals(default_com_position));
    CHECK_THAT(data.com_velocity, Equals(kVec3DZero));
    CHECK_THAT(data.com_acceleration, Equals(kVec3DZero));
    CHECK_THAT(data.zmp_position, Equals(kVec3DZero));
    CHECK_THAT(data.reaction_force, Equals(Vec3D(0, 0, G)));
    CHECK_THAT(data.external_force, Equals(kVec3DZero));
    CHECK_THAT(data.total_force, Equals(Vec3D(0, 0, G)));
  }

  SECTION("with COM position") {
    Fuzzer fuzz(0, 10);
    auto p0 = fuzz.get<Vec3D>();
    ComZmpModelData data(p0);
    CHECK(data.mass == default_mass);
    CHECK_THAT(data.nu, Equals(kVec3DZ));
    CHECK_THAT(data.com_position, Equals(p0));
    CHECK_THAT(data.com_velocity, Equals(kVec3DZero));
    CHECK_THAT(data.com_acceleration, Equals(kVec3DZero));
    CHECK_THAT(data.zmp_position, Equals(kVec3DZero));
    CHECK_THAT(data.reaction_force, Equals(Vec3D(0, 0, G)));
    CHECK_THAT(data.external_force, Equals(kVec3DZero));
    CHECK_THAT(data.total_force, Equals(Vec3D(0, 0, G)));
  }

  SECTION("with COM position and mass") {
    Fuzzer fuzz(0, 10);
    double m = fuzz.get();
    auto p0 = fuzz.get<Vec3D>();
    ComZmpModelData data(p0, m);
    CHECK(data.mass == m);
    CHECK_THAT(data.nu, Equals(kVec3DZ));
    CHECK_THAT(data.com_position, Equals(p0));
    CHECK_THAT(data.com_velocity, Equals(kVec3DZero));
    CHECK_THAT(data.com_acceleration, Equals(kVec3DZero));
    CHECK_THAT(data.zmp_position, Equals(kVec3DZero));
    CHECK_THAT(data.reaction_force, Equals(Vec3D(0, 0, m * G)));
    CHECK_THAT(data.external_force, Equals(kVec3DZero));
    CHECK_THAT(data.total_force, Equals(Vec3D(0, 0, m * G)));
  }
}

TEST_CASE("ComZmpModelData: copy constructor") {
  ComZmpModelData a;
  RandomizeData(&a);
  ComZmpModelData b(a);
  CheckData(a, b);
}

TEST_CASE("ComZmpModelData: copy assignment operator") {
  ComZmpModelData a, b;
  RandomizeData(&a);
  b = a;
  CheckData(a, b);
}

TEST_CASE("ComZmpModelData: move constructor") {
  double mass = 2.5;
  Vec3D com_pos = {2.0, 3.0, 4.0};
  ComZmpModelData a;
  a.mass = mass;
  a.com_position = com_pos;

  ComZmpModelData b = std::move(a);
  CHECK(b.mass == mass);
  CHECK_THAT(b.com_position, Equals(com_pos));

  auto f = [](ComZmpModelData arg) { return arg; };
  ComZmpModelData c = f(ComZmpModelData(com_pos, mass));
  CHECK(c.mass == mass);
}

TEST_CASE("ComZmpModelData: move assignment operator") {
  double mass = 2.5;
  Vec3D com_pos = {2.0, 3.0, 4.0};
  ComZmpModelData a, b, c;
  a.mass = mass;
  a.com_position = com_pos;

  b = std::move(a);
  CHECK(b.mass == mass);
  CHECK_THAT(b.com_position, Equals(com_pos));

  auto f = [](ComZmpModelData arg) { return arg; };
  c = f(ComZmpModelData(com_pos, mass));
  CHECK(c.mass == mass);
}

// ComZmpModelSystem class
TEST_CASE("ComZmpModelSystem: constructor",
          "[corelib][humanoid][ComZmpModelSystem]") {
  SECTION("constructor with data pointer") {
    auto data = createComZmpModelData();
    REQUIRE(data.use_count() == 1);
    ComZmpModelSystem sys(data);
    CHECK(sys.data_ptr().get() == data.get());
    CHECK(data.use_count() == 2);
  }
}

TEST_CASE("ComZmpModelSystem: accessors / mutators",
          "[corelib][humanoid][ComZmpModelSystem]") {
  auto data = createComZmpModelData();
  ComZmpModelSystem sys(data);

  SECTION("data pointer") {
    auto data2 = createComZmpModelData();
    sys.set_data_ptr(data2);
    CHECK(data.use_count() == 1);
    CHECK(sys.data_ptr().get() == data2.get());
    CHECK(data2.use_count() == 2);
  }
}

TEST_CASE("ComZmpModelSystem::operator() defines system of COM-ZMP model",
          "[corelib][humanoid][ComZmpModelSystem]") {
  Fuzzer fuzz;
  auto data = createComZmpModelData();
  ComZmpModelSystem sys(data);
  Vec3D p = {0, 1, 2};
  Vec3D v = {1, -2, -1};
  std::array<Vec3D, 2> x{{p, v}};
  std::array<Vec3D, 2> dxdt;
  double t = 0;

  SECTION("When function to compute acceleration is specified, use this") {
    auto a = fuzz.get<Vec3D>();
    auto f_acc = [a](const Vec3D&, const Vec3D&, const double) { return a; };
    sys.set_com_acceleration_f(f_acc);
    sys(x, dxdt, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == a);
  }

  SECTION("When function to compute reaction force is specified, use this") {
    auto f = fuzz.get<Vec3D>();
    auto f_force = [f](const Vec3D&, const Vec3D&, const double) { return f; };
    sys.set_reaction_force_f(f_force);
    sys(x, dxdt, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == f - Vec3D(0, 0, G));  // \ddot{p} = \frac{f}{m} - g

    SECTION("External force is also specified, use this as well") {
      auto ef = fuzz.get<Vec3D>();
      auto f_ef = [ef](const Vec3D&, const Vec3D&, const double) { return ef; };
      sys.set_external_force_f(f_ef);
      sys(x, dxdt, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] ==
            f + ef - Vec3D(0, 0, G));  // \ddot{p} = \frac{f + f_{e}}{m} - g
    }
  }

  SECTION("When external force is specified and not reaction force") {
    auto ef = fuzz.get<Vec3D>();
    auto f_ef = [ef](const Vec3D&, const Vec3D&, const double) { return ef; };
    sys.set_external_force_f(f_ef);
    sys(x, dxdt, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == ef);  // \ddot{p} = \frac{mg + f_{e}}{m} - g
  }

  SECTION("When ZMP position is specified, calculate acceleration from it") {
    Vec3D zmp = {1, -1, 0};
    auto f_zmp = [zmp](const Vec3D&, const Vec3D&, const double) {
      return zmp;
    };
    auto expected =
        computeComAcc(p, zmp, Vec3D(0, 0, data->mass * G), data->mass);
    sys.set_zmp_position_f(f_zmp);
    sys(x, dxdt, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == expected);
  }

  SECTION("When ZMP position and rection force are specified") {
    Vec3D zmp = {1, -1, 0};
    auto f_zmp = [zmp](const Vec3D&, const Vec3D&, const double) {
      return zmp;
    };
    Vec3D fz = {0, 0, 8};
    auto f_fz = [fz](const Vec3D&, const Vec3D&, const double) { return fz; };
    auto expected = computeComAcc(p, zmp, fz, data->mass);
    SECTION("Specify fz first then ZMP") {
      sys.set_reaction_force_f(f_fz);
      sys.set_zmp_position_f(f_zmp);
      sys(x, dxdt, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
    SECTION("Specify ZMP first then fz") {
      sys.set_zmp_position_f(f_zmp);
      sys.set_reaction_force_f(f_fz);
      sys(x, dxdt, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
  }

  SECTION("ZMP position, rection force and external force are specified") {
    Vec3D zmp = {1, -1, 0};
    auto f_zmp = [zmp](const Vec3D&, const Vec3D&, const double) {
      return zmp;
    };
    Vec3D fz = {0, 0, 8};
    auto f_fz = [fz](const Vec3D&, const Vec3D&, const double) { return fz; };
    Vec3D ef = fuzz.get<Vec3D>();
    auto f_ef = [ef](const Vec3D&, const Vec3D&, const double) { return ef; };
    auto expected = computeComAcc(p, zmp, fz, data->mass, ef);
    SECTION("Specify them in the following order: fz, ZMP, ef") {
      sys.set_reaction_force_f(f_fz);
      sys.set_zmp_position_f(f_zmp);
      sys.set_external_force_f(f_ef);
      sys(x, dxdt, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
    SECTION("Specify them in the following order: ZMP, fz, ef") {
      sys.set_zmp_position_f(f_zmp);
      sys.set_reaction_force_f(f_fz);
      sys.set_external_force_f(f_ef);
      sys(x, dxdt, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
    SECTION("Specify them in the following order: fz, ef, ZMP") {
      sys.set_reaction_force_f(f_fz);
      sys.set_external_force_f(f_ef);
      sys.set_zmp_position_f(f_zmp);
      sys(x, dxdt, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
  }

  SECTION("Mass value is not 1") {
    data->mass = 2.5;
    Vec3D zmp = {1, -1, 0};
    auto f_zmp = [zmp](const Vec3D&, const Vec3D&, const double) {
      return zmp;
    };
    Vec3D ef = fuzz.get<Vec3D>();
    auto f_ef = [ef](const Vec3D&, const Vec3D&, const double) { return ef; };
    auto expected =
        computeComAcc(p, zmp, Vec3D(0, 0, data->mass * G), data->mass, ef);
    sys.set_zmp_position_f(f_zmp);
    sys.set_external_force_f(f_ef);
    sys(x, dxdt, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == expected);
  }
}

TEST_CASE("ComZmpModelSystem::isZmpPositionSet() returns if it is set",
          "[corelib][humanoid][ComZmpModelSystem]") {
  auto data = createComZmpModelData();
  ComZmpModelSystem sys(data);
  CHECK_FALSE(sys.isZmpPositionSet());
  sys.set_zmp_position_f(
      [](const Vec3D&, const Vec3D&, const double) { return kVec3DZero; });
  CHECK(sys.isZmpPositionSet());
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

TEST_CASE("ComZmpModel::computeSqrZeta(double,double,double)",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("various COM height, ZMP height and COM acc fixed to 0") {
    double zz = 0;
    double az = 0;
    struct testcaset_t {
      double z;
      double expected_sqr_zeta;
    } testcases[] = {{1, G}, {G, 1.0}, {2, G / 2}, {4, G / 4}};
    for (const auto& c : testcases) {
      INFO("z=" << c.z << ", zz=" << zz << ", az=" << az);
      CHECK(computeSqrZeta(c.z, zz, az) == c.expected_sqr_zeta);
      CHECK(computeZeta(c.z, zz, az) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various ZMP height, COM height = 2 and COM acc = 0") {
    double z = 2;
    double az = 0;
    struct testcaset_t {
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, G}, {1.5, 2.0 * G}, {0.5, 2.0 * G / 3.0}, {-0.5, 2.0 * G / 5.0}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << c.zz << ", az=" << az);
      CHECK(computeSqrZeta(z, c.zz, az) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, c.zz, az) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various COM acceleration, COM height = 1.5 and ZMP height = 0.5") {
    double z = 1.5;
    double zz = 0.5;
    struct testcaset_t {
      double az;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, 1.0 + G}, {1.5, 1.5 + G}, {-1, -1.0 + G}, {G, 2.0 * G}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", az=" << c.az);
      CHECK(computeSqrZeta(z, zz, c.az) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, c.az) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("return 0 when (z - zz) <= 0") {
    double az = 0;
    struct testcase_t {
      double z;
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1, 0}, {1, 2, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << c.z << ", zz=" << c.zz << ", az=" << az);
      CHECK(computeSqrZeta(c.z, c.zz, az) == c.expected_sqr_zeta);
      CHECK(computeZeta(c.z, c.zz, az) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when (az + g) <= 0") {
    double z = 1;
    double zz = 0;
    struct testcase_t {
      double az;
      double expected_sqr_zeta;
    } testcases[] = {{-2.0 * G, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", az=" << c.az);
      CHECK(computeSqrZeta(z, zz, c.az) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, c.az) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }
}

TEST_CASE("ComZmpModel::computeSqrZeta(double,double,double,double)",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("various COM height, fixed ZMP = 0, Fz = G, m = 1") {
    double zz = 0;
    double fz = G;
    double m = 1;
    struct testcaset_t {
      double z;
      double expected_sqr_zeta;
    } testcases[] = {{1, G}, {G, 1.0}, {2, G / 2}, {4, G / 4}};
    for (const auto& c : testcases) {
      INFO("z=" << c.z << ", zz=" << zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(c.z, zz, fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(c.z, zz, fz, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various ZMP height, fixed COM = 2, Fz = G, m = 1") {
    double z = 2;
    double fz = G;
    double m = 1;
    struct testcaset_t {
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, G}, {1.5, 2.0 * G}, {0.5, 2.0 * G / 3.0}, {-0.5, 2.0 * G / 5.0}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << c.zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(z, c.zz, fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, c.zz, fz, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various Fz, fixed COM = 2, ZMP = 1, m = 1") {
    double z = 2;
    double zz = 1;
    double m = 1;
    struct testcaset_t {
      double fz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1}, {1.5, 1.5}, {0.5, 0.5}, {G, G}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", fz=" << c.fz << ", m=" << m);
      CHECK(computeSqrZeta(z, zz, c.fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, c.fz, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various mass, fixed COM = 2, ZMP = 1, Fz = 2") {
    double z = 2;
    double zz = 1;
    double fz = 2;
    struct testcaset_t {
      double m;
      double expected_sqr_zeta;
    } testcases[] = {{1, 2}, {2, 1}, {0.5, 4}, {1.5, 4.0 / 3}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", fz=" << fz << ", m=" << c.m);
      CHECK(computeSqrZeta(z, zz, fz, c.m) == Approx(c.expected_sqr_zeta));
      CHECK(computeZeta(z, zz, fz, c.m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("return 0 when (z - zz) <= 0") {
    double fz = 1;
    double m = 1;
    struct testcase_t {
      double z;
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1, 0}, {1, 2, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << c.z << ", zz=" << c.zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(c.z, c.zz, fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(c.z, c.zz, fz, m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when fz < 0") {
    double z = 2;
    double zz = 1;
    double m = 1;
    struct testcase_t {
      double fz;
      double expected_sqr_zeta;
    } testcases[] = {{-1, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", fz=" << c.fz << ", m=" << m);
      CHECK(computeSqrZeta(z, zz, c.fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, c.fz, m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when mass < 0") {
    double z = 2;
    double zz = 1;
    double fz = 1;
    struct testcase_t {
      double m;
      double expected_sqr_zeta;
    } testcases[] = {{-1, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", fz=" << fz << ", m=" << c.m);
      CHECK(computeSqrZeta(z, zz, fz, c.m) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, fz, c.m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }
}

TEST_CASE("ComZmpModel::computeSqrZeta(const Vec3D&,const Vec3D&,const Vec3D&)",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("various COM height, ZMP height and COM acc fixed to 0") {
    double zz = 0;
    double az = 0;
    struct testcaset_t {
      double z;
      double expected_sqr_zeta;
    } testcases[] = {{1, G}, {G, 1.0}, {2, G / 2}, {4, G / 4}};
    for (const auto& c : testcases) {
      Vec3D p{1, 2, c.z};
      Vec3D pz{0, 1, zz};
      Vec3D ddp{1, 0, az};
      INFO("z=" << c.z << ", zz=" << zz << ", az=" << az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various ZMP height, COM height = 2 and COM acc = 0") {
    double z = 2;
    double az = 0;
    struct testcaset_t {
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, G}, {1.5, 2.0 * G}, {0.5, 2.0 * G / 3.0}, {-0.5, 2.0 * G / 5.0}};
    for (const auto& c : testcases) {
      Vec3D p{1, 0, z};
      Vec3D pz{1, 1, c.zz};
      Vec3D ddp{1, 2, az};
      INFO("z=" << z << ", zz=" << c.zz << ", az=" << az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various COM acceleration, COM height = 1.5 and ZMP height = 0.5") {
    double z = 1.5;
    double zz = 0.5;
    struct testcaset_t {
      double az;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, 1.0 + G}, {1.5, 1.5 + G}, {-1, -1.0 + G}, {G, 2.0 * G}};
    for (const auto& c : testcases) {
      Vec3D p{1, 0.1, z};
      Vec3D pz{0, 0.1, zz};
      Vec3D ddp{1, 0.1, c.az};
      INFO("z=" << z << ", zz=" << zz << ", az=" << c.az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("return 0 when (z - zz) <= 0") {
    double az = 0;
    struct testcase_t {
      double z;
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1, 0}, {1, 2, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, c.z};
      Vec3D pz{0, 0, c.zz};
      Vec3D ddp{0, 0, az};
      INFO("z=" << c.z << ", zz=" << c.zz << ", az=" << az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when (az + g) <= 0") {
    double z = 1;
    double zz = 0;
    struct testcase_t {
      double az;
      double expected_sqr_zeta;
    } testcases[] = {{-2.0 * G, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, z};
      Vec3D pz{0, 0, zz};
      Vec3D ddp{0, 0, c.az};
      INFO("z=" << z << ", zz=" << zz << ", az=" << c.az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }
}

TEST_CASE(
    "ComZmpModel::computeSqrZeta(const Vec3D&,const Vec3D&,const Vec3D&,const "
    "Vec3D&)",
    "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("various COM height, fixed ZMP = 0, Fz = G, m = 1") {
    double zz = 0;
    double fz = G;
    double m = 1;
    struct testcaset_t {
      double z;
      double expected_sqr_zeta;
    } testcases[] = {{1, G}, {G, 1.0}, {2, G / 2}, {4, G / 4}};
    for (const auto& c : testcases) {
      Vec3D p{1, 2, c.z};
      Vec3D pz{0, 1, zz};
      Vec3D f{1, 0, fz};
      INFO("z=" << c.z << ", zz=" << zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various ZMP height, fixed COM = 2, Fz = G, m = 1") {
    double z = 2;
    double fz = G;
    double m = 1;
    struct testcaset_t {
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, G}, {1.5, 2.0 * G}, {0.5, 2.0 * G / 3.0}, {-0.5, 2.0 * G / 5.0}};
    for (const auto& c : testcases) {
      Vec3D p{0, 1, z};
      Vec3D pz{1, 1, c.zz};
      Vec3D f{0, 2, fz};
      INFO("z=" << z << ", zz=" << c.zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various Fz, fixed COM = 2, ZMP = 1, m = 1") {
    double z = 2;
    double zz = 1;
    double m = 1;
    struct testcaset_t {
      double fz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1}, {1.5, 1.5}, {0.5, 0.5}, {G, G}};
    for (const auto& c : testcases) {
      Vec3D p{-1, 0, z};
      Vec3D pz{0.1, 0, zz};
      Vec3D f{-0.1, 0, c.fz};
      INFO("z=" << z << ", zz=" << zz << ", fz=" << c.fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various mass, fixed COM = 2, ZMP = 1, Fz = 2") {
    double z = 2;
    double zz = 1;
    double fz = 2;
    struct testcaset_t {
      double m;
      double expected_sqr_zeta;
    } testcases[] = {{1, 2}, {2, 1}, {0.5, 4}, {1.5, 4.0 / 3}};
    for (const auto& c : testcases) {
      Vec3D p{1, -1, z};
      Vec3D pz{-1, 1, zz};
      Vec3D f{0, -1, fz};
      INFO("z=" << z << ", zz=" << zz << ", fz=" << fz << ", m=" << c.m);
      CHECK(computeSqrZeta(p, pz, f, c.m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, c.m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("return 0 when (z - zz) <= 0") {
    double fz = 1;
    double m = 1;
    struct testcase_t {
      double z;
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1, 0}, {1, 2, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, c.z};
      Vec3D pz{0, 0, c.zz};
      Vec3D f{0, 0, fz};
      INFO("z=" << c.z << ", zz=" << c.zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when fz < 0") {
    double z = 2;
    double zz = 1;
    double m = 1;
    struct testcase_t {
      double fz;
      double expected_sqr_zeta;
    } testcases[] = {{-1, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, z};
      Vec3D pz{0, 0, zz};
      Vec3D f{0, 0, c.fz};
      INFO("z=" << z << ", zz=" << zz << ", fz=" << c.fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when mass < 0") {
    double z = 2;
    double zz = 1;
    double fz = 1;
    struct testcase_t {
      double m;
      double expected_sqr_zeta;
    } testcases[] = {{-1, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, z};
      Vec3D pz{0, 0, zz};
      Vec3D f{0, 0, fz};
      INFO("z=" << z << ", zz=" << zz << ", fz=" << fz << ", m=" << c.m);
      CHECK(computeSqrZeta(p, pz, f, c.m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, c.m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }
}

TEST_CASE("ComZmpModel::computeReactForce(const Vec3D&,double)",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("case: various COM acceleration with fixed mass = 1") {
    double mass = 1;
    struct testcase_t {
      Vec3D acc;
      Vec3D expected_force;
    } testcases[] = {{{0, 0, -G}, {0, 0, 0}},
                     {{1, 2, 3}, {1, 2, 3 + G}},
                     {{-1, -3, G}, {-1, -3, 2 * G}}};
    for (const auto& c : testcases) {
      INFO("m = " << mass << ", acc = " << c.acc);
      CHECK(computeReactForce(c.acc, mass) == c.expected_force);
    }
  }

  SECTION("case: various mass with fixed acceleration = (-1, 1, -0.5g)") {
    Vec3D acc(-1, 1, -0.5 * G);
    struct testcase_t {
      double mass;
      Vec3D expected_force;
    } testcases[] = {
        {1, {-1, 1, 0.5 * G}}, {3, {-3, 3, 1.5 * G}}, {5, {-5, 5, 2.5 * G}}};
    for (const auto& c : testcases) {
      INFO("m = " << c.mass << ", acc = " << acc);
      CHECK(computeReactForce(acc, c.mass) == c.expected_force);
    }
  }
}

TEST_CASE(
    "ComZmpModel::computeReactForce(const Vec3D&,const Vec3D&,double,double)",
    "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("mass = 1, zeta2 = G, various COM / ZMP position") {
    double mass = 1;
    double zeta2 = G;
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      Vec3D expected_force;
    } testcases[] = {{{0, 0, 1}, {0, 0, 0}, {0, 0, G}},
                     {{1, 2, 2}, {-1, -1, 1}, {2 * G, 3 * G, G}},
                     {{-1, 0.5, 1}, {1, 0.5, 0}, {-2 * G, 0, G}}};
    for (const auto& c : testcases) {
      INFO("m = " << mass << ", zeta2 = " << zeta2 << ", com = " << c.com
                  << ", zmp = " << c.zmp);
      CHECK(computeReactForce(c.com, c.zmp, zeta2, mass) == c.expected_force);
    }
  }

  SECTION("COM = (1, -1, 1), ZMP = (0, 0, 0), various mass and zeta2") {
    Vec3D com(1, -1, 1);
    Vec3D zmp(0, 0, 0);
    struct testcase_t {
      double mass;
      double zeta2;
      Vec3D expected_force;
    } testcases[] = {{1, 2, {2, -2, 2}},
                     {0.5, 1, {0.5, -0.5, 0.5}},
                     {3, G, {3 * G, -3 * G, 3 * G}}};
    for (const auto& c : testcases) {
      INFO("m = " << c.mass << ", zeta2 = " << c.zeta2 << ", com = " << com
                  << ", zmp = " << zmp);
      CHECK(computeReactForce(com, zmp, c.zeta2, c.mass) == c.expected_force);
    }
  }
}

TEST_CASE(
    "ComZmpModel::computeReactForce(const Vec3D&,const Vec3D&,const "
    "Vec3D&,double)",
    "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  struct testcase_t {
    Vec3D com_p;
    Vec3D zmp_p;
    Vec3D com_a;
    double mass;
    Vec3D expected_force;
  } testcases[] = {{{0, 0, 1}, {0, 0, 0}, {0, 0, 0}, 1, {0, 0, G}},
                   {{1, 2, 2}, {0, -1, 0}, {1, 0, G}, 1, {G, 3. * G, 2. * G}},
                   {{-1, 0.5, 2},
                    {1, 1, 0.5},
                    {0, 0, -0.5 * G},
                    2,
                    {-4. * G / 3, -G / 3, G}}};
  for (const auto& c : testcases) {
    INFO("m = " << c.mass << ", com = " << c.com_p << ", zmp = " << c.zmp_p
                << ", com acc = " << c.com_a);
    CHECK(computeReactForce(c.com_p, c.zmp_p, c.com_a, c.mass) ==
          c.expected_force);
  }
}

TEST_CASE("ComZmpModel::computeReactForce(const Vec3D&,const Vec3D&,double)",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  struct testcase_t {
    Vec3D com_p;
    Vec3D zmp_p;
    double fz;
    Vec3D expected_force;
  } testcases[] = {{{0, 0, 1}, {0, 0, 0}, 1, {0, 0, 1}},
                   {{0, 0, 1}, {0, 0, 0}, 1.5, {0, 0, 1.5}},
                   {{1, -1, 2}, {0, 1, 0.5}, 1.1, {2.2 / 3, -4.4 / 3, 1.1}},
                   {{1, -1, 2}, {0, 1, 0.5}, 30, {20, -40, 30}},
                   {{-0.5, 1.9, 1.4}, {1, -1.1, -0.1}, 1, {-1, 2, 1}},
                   {{-0.5, 1.9, 1.4}, {1, -1.1, -0.1}, 10, {-10, 20, 10}}};
  for (const auto& c : testcases) {
    INFO("com = " << c.com_p << ", zmp = " << c.zmp_p << ", fz = " << c.fz);
    CHECK(computeReactForce(c.com_p, c.zmp_p, c.fz) == c.expected_force);
  }
}

TEST_CASE("ComZmpModel::computeComAcc(const Vec3D&,double)",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("without additional force") {
    struct testcase_t {
      Vec3D f;
      double m;
      Vec3D expected_acc;
    } testcases[] = {{{1, 1, 0}, 1, {1, 1, -G}},
                     {{-1, 2, G}, 2, {-0.5, 1, -0.5 * G}},
                     {{0, -1, -0.5 * G}, 1.5, {0, -2. / 3, -4. * G / 3}}};
    for (const auto& c : testcases) {
      INFO("m = " << c.m << ", f = " << c.f);
      CHECK(computeComAcc(c.f, c.m) == c.expected_acc);
    }
  }
  SECTION("with additional force") {
    struct testcase_t {
      Vec3D f;
      double m;
      Vec3D ef;
      Vec3D expected_acc;
    } testcases[] = {
        {{1, 1, 0}, 1, {0, 1, 1}, {1, 2, -G + 1}},
        {{-1, 2, G}, 2, {-2, 2, 2 * G}, {-1.5, 2, 0.5 * G}},
        {{0, -1, -0.5 * G}, 1.5, {0.5, -0.5, 0.5 * G}, {1. / 3, -1, -G}}};
    for (const auto& c : testcases) {
      INFO("m = " << c.m << ", f = " << c.f << ", added f = " << c.ef);
      CHECK(computeComAcc(c.f, c.m, c.ef) == c.expected_acc);
    }
  }
}

TEST_CASE("ComZmpModel::computeComAcc(const Vec3D&,const Vec3D&,double)",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("without additional force") {
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      double zeta2;
      Vec3D expected_acc;
    } testcases[] = {{{0, 0, 1}, {0, 0, 0}, G, {0, 0, 0}},
                     {{1, 2, 2.5}, {-1, 1, 0.5}, 2, {4, 2, 4. - G}},
                     {{-1, 1.5, 2}, {0.5, 1.5, 0}, 1.5, {-2.25, 0, 3. - G}}};
    for (const auto& c : testcases) {
      INFO("zeta2 = " << c.zeta2 << ", com = " << c.com << ", zmp = " << c.zmp);
      CHECK(computeComAcc(c.com, c.zmp, c.zeta2) == c.expected_acc);
    }
  }
  SECTION("with additional force") {
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      double zeta2;
      double m;
      Vec3D ef;
      Vec3D expected_acc;
    } testcases[] = {
        {{0, 0, 1}, {0, 0, 0}, G, 1, {1, 1, 1}, {1, 1, 1}},
        {{1, 2, 2.5}, {-1, 1, 0.5}, 2, 1.5, {1.5, -1.5, 3}, {5, 1, 6. - G}},
        {{-1, 1.5, 2}, {0.5, 1.5, 0}, 1.5, 1, {-1, 2, 3}, {-3.25, 2, 6. - G}}};
    for (const auto& c : testcases) {
      INFO("zeta2 = " << c.zeta2 << ", com = " << c.com << ", zmp = " << c.zmp
                      << ", mass = " << c.m << ", ef = " << c.ef);
      CHECK(computeComAcc(c.com, c.zmp, c.zeta2, c.m, c.ef) == c.expected_acc);
    }
  }
}

TEST_CASE(
    "ComZmpModel::computeComAcc(const Vec3D&,const Vec3D&,const "
    "Vec3D&,double)",
    "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("without additional force") {
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      double m;
      Vec3D f;
      Vec3D expected_acc;
    } testcases[] = {
        {{0, 0, 1}, {0, 0, 0}, 1, {0, 0, G}, {0, 0, 0}},
        {{1, 2, 2.5},
         {-1, 0, 0.5},
         1.5,
         {0, 0, 1},
         {2. / 3, 2. / 3, 2. / 3 - G}},
        {{-1, 1.5, 2}, {0.5, -1.5, 0}, 2, {0, 0, 2}, {-3. / 4, 3. / 2, 1 - G}}};
    for (const auto& c : testcases) {
      INFO("com = " << c.com << ", zmp = " << c.zmp << ", mass = " << c.m
                    << ", f = " << c.f);
      CHECK(computeComAcc(c.com, c.zmp, c.f, c.m) == c.expected_acc);
    }
  }
  SECTION("with additional force") {
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      double m;
      Vec3D f;
      Vec3D ef;
      Vec3D expected_acc;
    } testcases[] = {{{0, 0, 1}, {0, 0, 0}, 1, {0, 0, G}, {1, 2, 3}, {1, 2, 3}},
                     {{1, 2, 2.5},
                      {-1, 0, 0.5},
                      1.5,
                      {0, 0, 1},
                      {0.5, -0.5, 0.5},
                      {1, 1. / 3, 1 - G}},
                     {{-1, 1.5, 2},
                      {0.5, -1.5, 0},
                      2,
                      {0, 0, 2},
                      {-1, 1, G},
                      {-5. / 4, 2, 1 - 0.5 * G}}};
    for (const auto& c : testcases) {
      INFO("com = " << c.com << ", zmp = " << c.zmp << ", mass = " << c.m
                    << ", f = " << c.f << ", ef = " << c.ef);
      CHECK(computeComAcc(c.com, c.zmp, c.f, c.m, c.ef) == c.expected_acc);
    }
  }
}

TEST_CASE("compute the COM acceleration based on COM-ZMP model",
          "[corelib][humanoid]") {
  ComZmpModel model;
  Vec3D force = model.data().reaction_force;
  double mass = model.data().mass;

  SECTION("case: the COM height is assumed to be const, namely zeta is const") {
    struct testcase_t {
      Vec3D com_pos;
      Vec3D zmp_pos;
      Vec3D expected_acc;
    } testcases[] = {
        // cases where the COM height equals to G, namely zeta equals to 1.
        {{0, 0, G}, {0, 0, 0}, {0, 0, 0}},
        {{1, 0, G}, {0, 0, 0}, {1, 0, 0}},
        {{3, 0, G}, {0, 0, 0}, {3, 0, 0}},
        {{0, 0, G}, {1, 0, 0}, {-1, 0, 0}},
        {{0, 0, G}, {3, 0, 0}, {-3, 0, 0}},
        {{0, 2, G}, {0, 0, 0}, {0, 2, 0}},
        {{0, 4, G}, {0, 0, 0}, {0, 4, 0}},
        {{0, 0, G}, {0, 2, 0}, {0, -2, 0}},
        {{0, 0, G}, {0, 4, 0}, {0, -4, 0}},
        {{3, 1, G}, {2, 2, 0}, {1, -1, 0}},
        {{1, 3, G}, {-1, 2, 0}, {2, 1, 0}},
        // cases where the COM height equals to 1
        {{0, 0, 1}, {0, 0, 0}, {0, 0, 0}},
        {{1, 0, 1}, {0, 0, 0}, {G, 0, 0}},
        {{3, 0, 1}, {0, 0, 0}, {3 * G, 0, 0}},
        {{0, 0, 1}, {1, 0, 0}, {-G, 0, 0}},
        {{0, 0, 1}, {3, 0, 0}, {-3 * G, 0, 0}},
        {{0, 2, 1}, {0, 0, 0}, {0, 2 * G, 0}},
        {{0, 4, 1}, {0, 0, 0}, {0, 4 * G, 0}},
        {{0, 0, 1}, {0, 2, 0}, {0, -2 * G, 0}},
        {{0, 0, 1}, {0, 4, 0}, {0, -4 * G, 0}},
        {{3, 1, 1}, {2, 2, 0}, {G, -G, 0}},
        {{1, 3, 1}, {-1, 2, 0}, {2 * G, G, 0}},
        // given random values
        {{2, 3, 2}, {-2, -1, 0}, {2 * G, 2 * G, 0}},
        {{1, 3, 0.5}, {-1, -1, 0}, {4 * G, 8 * G, 0}},
    };

    for (auto c : testcases) {
      Vec3D acc = computeComAcc(c.com_pos, c.zmp_pos, force, mass);
      CAPTURE(&c.com_pos);
      CAPTURE(&c.zmp_pos);
      CHECK_THAT(acc, Equals(c.expected_acc));
    }
  }
}

TEST_CASE("modify step time after calling update with double type",
          "[corelib][humanoid]") {
  ComZmpModel model;
  Fuzzer fuzz(0.0001, 0.1);
  double dt1 = fuzz.get();
  double dt2 = fuzz.get();

  REQUIRE(model.time_step() != dt1);
  model.update(dt1);
  CHECK(model.time_step() == dt1);
  model.update();
  CHECK(model.time_step() == dt1);

  REQUIRE(model.time_step() != dt2);
  model.update(dt2);
  CHECK(model.time_step() == dt2);
  model.update();
  CHECK(model.time_step() == dt2);
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
      model.setZmpPos(c.zmp_pos);
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
      model.setZmpPos(zmp_pos);
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
  model.setZmpPos(kVec3DZero);
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
  model.setZmpPos(desired_zmp, desired_fz);
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
  model.setZmpPos(desired_zmp, desired_fz);
  model.setExternalForce(ext_force);
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
  model.setExternalForce(ext_force);
  REQUIRE(data->reaction_force == Vec3D(0, 0, G));
  model.update();
  CHECK(data->total_force == Vec3D(1.5, -1.5, G - 10));
  CHECK(data->com_acceleration == Vec3D(1.5, -1.5, -10));
  CHECK(data->external_force == ext_force);
}

}  // namespace
}  // namespace holon
