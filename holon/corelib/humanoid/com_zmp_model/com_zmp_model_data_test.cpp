/* com_zmp_model_data - Data for COM-ZMP model
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

#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_data.hpp"

#include <roki/rk_g.h>

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

}  // namespace
}  // namespace holon
