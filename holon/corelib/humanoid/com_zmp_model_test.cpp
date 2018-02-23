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
  }

  SECTION("with parameters") {
    Fuzzer fuzz(0, 10);
    for (auto i = 0; i < 10; ++i) {
      double m = fuzz.get();
      ComZmpModelData data(m);
      CHECK(data.mass == m);
      CHECK_THAT(data.nu, Equals(kVec3DZ));
      CHECK_THAT(data.com_position, Equals(default_com_position));
      CHECK_THAT(data.com_velocity, Equals(kVec3DZero));
      CHECK_THAT(data.com_acceleration, Equals(kVec3DZero));
      CHECK_THAT(data.zmp_position, Equals(kVec3DZero));
      CHECK_THAT(data.reaction_force, Equals(Vec3D(0, 0, m * G)));
      CHECK_THAT(data.external_force, Equals(kVec3DZero));
    }
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
  ComZmpModelData c = f(ComZmpModelData(mass));
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
  c = f(ComZmpModelData(mass));
  CHECK(c.mass == mass);
}

// ComZmpModel class
TEST_CASE("ComZmpModel constructor", "[corelib][humanoid][ComZmpModel]") {
  double default_mass = 1;
  Vec3D default_com_position = {0, 0, 1};

  SECTION("default constructor (no parameters)") {
    ComZmpModel model;
    CHECK(model.data().mass == default_mass);
    CHECK(model.data().com_position == default_com_position);
  }

  SECTION("ComZmpModel(double t_mass)") {
    Fuzzer fuzz(0, 10);
    for (auto i = 0; i < 10; ++i) {
      double m = fuzz.get();
      ComZmpModel model(m);
      CHECK(model.data().mass == m);
    }

    SECTION("mass should be positive") {
      zEchoOff();
      ComZmpModel model1(-1.0);
      CHECK(model1.data().mass == 1.0);
      ComZmpModel model2(0.0);
      CHECK(model2.data().mass == 1.0);
      zEchoOn();
    }
  }

  SECTION("ComZmpModel(Data t_data)") {
    auto data = ComZmpModelDataFactory();
    ComZmpModel model(data);
    CHECK(&model.data() == data.get());
  }
}

TEST_CASE("ComZmpModel: accessor/mutator of data",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("ComZmpModel can hold another Data pointer") {
    auto data1 = ComZmpModelDataFactory();
    REQUIRE(&model.data() != data1.get());
    model.set_data_ptr(data1);
    CHECK(&model.data() == data1.get());

    auto data2 = ComZmpModelDataFactory();
    REQUIRE(&model.data() != data2.get());
    model.set_data_ptr(data2);
    CHECK(&model.data() == data2.get());
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

TEST_CASE("ComZmpModel::set_external_force() set external force",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;
  Fuzzer fuzz;
  Vec3D v = fuzz.get<Vec3D>();
  model.set_external_force(v);
  CHECK(model.data().external_force == v);

  model.clear_external_force();
  CHECK(model.data().external_force == kVec3DZero);
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
    auto data = ComZmpModelDataFactory(2);
    ComZmpModel model(data);
    CHECK(model.mass() == 2.0);
    data->mass = 20;
    CHECK(model.mass() == 20.0);
  }
  SECTION("case 3") {
    ComZmpModel model;
    auto data = ComZmpModelDataFactory(3);
    model.set_data_ptr(data);
    CHECK(model.mass() == 3.0);
    data->mass = 30;
    CHECK(model.mass() == 30.0);
  }
}

SCENARIO("ComZmpModel: function to reset COM position",
         "[corelib][humanoid][ComZmpModel]") {
  GIVEN("initialize with random values") {
    auto data = ComZmpModelDataFactory();
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
  }
  SECTION("ComZmpModel::copy_data(const Data&)") {
    ComZmpModelData data;
    RandomizeData(&data);

    ComZmpModel model;
    model.copy_data(data);
    CheckData(model.data(), data);
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
      CHECK(model.computeSqrZeta(c.z, zz, az) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(c.z, zz, az) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(z, c.zz, az) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(z, c.zz, az) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(z, zz, c.az) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(z, zz, c.az) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(c.z, c.zz, az) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(c.z, c.zz, az) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(z, zz, c.az) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(z, zz, c.az) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(c.z, zz, fz, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(c.z, zz, fz, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(z, c.zz, fz, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(z, c.zz, fz, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(z, zz, c.fz, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(z, zz, c.fz, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(z, zz, fz, c.m) ==
            Approx(c.expected_sqr_zeta));
      CHECK(model.computeZeta(z, zz, fz, c.m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(c.z, c.zz, fz, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(c.z, c.zz, fz, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(z, zz, c.fz, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(z, zz, c.fz, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(z, zz, fz, c.m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(z, zz, fz, c.m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, f, c.m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, f, c.m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeSqrZeta(p, pz, f, c.m) == c.expected_sqr_zeta);
      CHECK(model.computeZeta(p, pz, f, c.m) == sqrt(c.expected_sqr_zeta));
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
      CHECK(model.computeReactForce(c.acc, mass) == c.expected_force);
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
      CHECK(model.computeReactForce(acc, c.mass) == c.expected_force);
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
      CHECK(model.computeReactForce(c.com, c.zmp, zeta2, mass) ==
            c.expected_force);
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
      CHECK(model.computeReactForce(com, zmp, c.zeta2, c.mass) ==
            c.expected_force);
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
    CHECK(model.computeReactForce(c.com_p, c.zmp_p, c.com_a, c.mass) ==
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
    CHECK(model.computeReactForce(c.com_p, c.zmp_p, c.fz) == c.expected_force);
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
      CHECK(model.computeComAcc(c.f, c.m) == c.expected_acc);
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
      CHECK(model.computeComAcc(c.f, c.m, c.ef) == c.expected_acc);
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
      CHECK(model.computeComAcc(c.com, c.zmp, c.zeta2) == c.expected_acc);
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
      CHECK(model.computeComAcc(c.com, c.zmp, c.zeta2, c.m, c.ef) ==
            c.expected_acc);
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
      CHECK(model.computeComAcc(c.com, c.zmp, c.f, c.m) == c.expected_acc);
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
      CHECK(model.computeComAcc(c.com, c.zmp, c.f, c.m, c.ef) ==
            c.expected_acc);
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
      Vec3D acc = model.computeComAcc(c.com_pos, c.zmp_pos, force, mass);
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

TEST_CASE("ComZmpModel::inputZmpPos computes reaction force from ZMP position",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;
  SECTION("COM height assumed to be constant") {
    Vec3D pz = {1, -1, 0};
    Vec3D expected_f =
        model.computeReactForce(model.data().com_position, pz, G);
    model.inputZmpPos(pz);
    CHECK(model.data().reaction_force == expected_f);
  }
  SECTION("COM moves along vertical direction as well") {
    Vec3D pz = {-0.5, 1.2, -0.1};
    double fz = 10;
    Vec3D expected_f =
        model.computeReactForce(model.data().com_position, pz, fz);
    model.inputZmpPos(pz, fz);
    CHECK(model.data().zmp_position == pz);
    CHECK(model.data().reaction_force == expected_f);
  }
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
      Vec3D f = model.computeReactForce(c.com_pos, c.zmp_pos, model.mass() * G);
      Vec3D expected_com_acc = model.computeComAcc(f, model.mass());

      model.data_ptr()->com_position = c.com_pos;
      model.data_ptr()->com_velocity = c.com_vel;
      model.inputZmpPos(c.zmp_pos);
      model.update();
      CHECK_THAT(model.data().com_acceleration, Equals(expected_com_acc));
    }
  }
}

SCENARIO("update COM position, velocity, acceleration", "[corelib][humanoid]") {
  GIVEN("COM stays at (0, 0, 1)") {
    auto data = ComZmpModelDataFactory();
    ComZmpModel model(data);

    WHEN("input ZMP position as (-1, -0.5, 0) and update") {
      Vec3D zmp_pos = {-1, -0.5, 0};
      model.inputZmpPos(zmp_pos);
      REQUIRE(model.update());

      THEN("horizontal velocity should be positive") {
        Vec3D vel = model.data().com_velocity;
        CHECK(vel.x() > 0.0);
        CHECK(vel.y() > 0.0);
      }
      THEN("horizontal position should still be at zero") {
        Vec3D pos = model.data().com_position;
        CHECK(pos.x() == 0.0);
        CHECK(pos.y() == 0.0);
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
  auto data = ComZmpModelDataFactory();
  ComZmpModel model(data);
  Vec3D p = {0, 0, 0};

  data->com_position = p;
  zEchoOff();
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
  model.inputZmpPos(desired_zmp, desired_fz);
  model.update();
  CHECK(data->reaction_force == Vec3D(10. / 0.42, -10, desired_fz));
  CHECK(data->com_acceleration == Vec3D(10. / 0.42, -10, 10 - G));
}

}  // namespace
}  // namespace holon
