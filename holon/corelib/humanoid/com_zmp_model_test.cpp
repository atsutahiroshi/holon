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
  a.mass = 1.5;
  a.com_position = Vec3D(1.0, 2.0, 3.0);
  ComZmpModelData b(a);
  CHECK(b.mass == a.mass);
  CHECK_THAT(b.com_position, Equals(a.com_position));
}

TEST_CASE("ComZmpModelData: copy assignment operator") {
  ComZmpModelData a, b;
  a.mass = 1.5;
  a.com_position = Vec3D(1.0, 2.0, 3.0);

  b = a;
  CHECK(b.mass == a.mass);
  CHECK_THAT(b.com_position, Equals(a.com_position));
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
    auto data = std::make_shared<ComZmpModelData>();
    ComZmpModel model(data);
    CHECK(&model.data() == data.get());
  }
}

TEST_CASE("ComZmpModel: accessor/mutator of data",
          "[corelib][humanoid][ComZmpModel]") {
  ComZmpModel model;

  SECTION("ComZmpModel can hold another Data pointer") {
    auto data1 = std::make_shared<ComZmpModelData>();
    REQUIRE(&model.data() != data1.get());
    model.set_data(data1);
    CHECK(&model.data() == data1.get());

    auto data2 = std::make_shared<ComZmpModelData>();
    REQUIRE(&model.data() != data2.get());
    model.set_data(data2);
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

SCENARIO("ComZmpModel: function to reset COM position",
         "[corelib][humanoid][ComZmpModel]") {
  GIVEN("initialize with random values") {
    auto data = std::make_shared<ComZmpModelData>();
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

TEST_CASE("test if acceleration is modified after update",
          "[corelib][humanoid]") {
  auto data = std::make_shared<ComZmpModelData>();
  ComZmpModel model(data);
  Vec3D force = model.data().reaction_force;
  double mass = model.data().mass;

  SECTION("update acceleration") {
    struct testcase_t {
      Vec3D com_pos;
      Vec3D com_vel;
      Vec3D zmp_pos;
    } testcases[] = {{{0, 0, 1}, {0, 0, 0}, {1, 0, 0}},
                     {{0, 0.1, 1}, {0.1, -0.1, 0}, {0.2, 0.1, 0}}};
    for (auto& c : testcases) {
      Vec3D expected_com_acc =
          model.computeComAcc(c.com_pos, c.zmp_pos, force, mass);

      data->com_position = c.com_pos;
      data->com_velocity = c.com_vel;
      data->zmp_position = c.zmp_pos;
      model.update();
      CHECK_THAT(model.data().com_acceleration, Equals(expected_com_acc));
    }
  }
}

SCENARIO("update COM position, velocity, acceleration", "[corelib][humanoid]") {
  GIVEN("COM stays at (0, 0, 1)") {
    auto data = std::make_shared<ComZmpModelData>();
    ComZmpModel model(data);

    WHEN("input ZMP position as (-1, -0.5, 0) and update") {
      Vec3D zmp_pos = {-1, -0.5, 0};
      data->zmp_position = zmp_pos;
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
  auto data = std::make_shared<ComZmpModelData>();
  ComZmpModel model(data);
  Vec3D p = {0, 0, 0};

  data->com_position = p;
  zEchoOff();
  CHECK_FALSE(model.update());
  zEchoOn();
}

}  // namespace
}  // namespace holon
