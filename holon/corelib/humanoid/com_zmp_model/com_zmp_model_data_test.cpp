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

using experimental::ComZmpModelData;

void RandomizeData(ComZmpModelData data) {
  Fuzzer fuzz;
  Fuzzer positive(0, 10);

  data.get<0>().mass = positive();
  data.get<0>().nu = fuzz.get<Vec3D>();
  data.get<0>().com_position = fuzz.get<Vec3D>();
  data.get<0>().com_position.set_z(positive());
  data.get<0>().com_velocity = fuzz.get<Vec3D>();
  data.get<0>().com_acceleration = fuzz.get<Vec3D>();
  data.get<0>().zmp_position = fuzz.get<Vec3D>();
  data.get<0>().reaction_force = fuzz.get<Vec3D>();
  data.get<0>().external_force = fuzz.get<Vec3D>();
  data.get<0>().total_force = fuzz.get<Vec3D>();
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

TEST_CASE("ComZmpModelData: constructor",
          "[corelib][humanoid][ComZmpModelData]") {
  double default_mass = 1;
  Vec3D default_com_position = {0, 0, 1};

  SECTION("default constructor (no parameters)") {
    ComZmpModelData data;
    CHECK(data().mass == default_mass);
    CHECK(data().nu == kVec3DZ);
    CHECK(data().com_position == default_com_position);
    CHECK(data().com_velocity == kVec3DZero);
    CHECK(data().com_acceleration == kVec3DZero);
    CHECK(data().zmp_position == kVec3DZero);
    CHECK(data().reaction_force == Vec3D(0, 0, G));
    CHECK(data().external_force == kVec3DZero);
    CHECK(data().total_force == Vec3D(0, 0, G));
  }

  SECTION("with COM position") {
    Fuzzer fuzz(0, 10);
    auto p0 = fuzz.get<Vec3D>();
    ComZmpModelData data(p0);
    CHECK(data().mass == default_mass);
    CHECK(data().nu == kVec3DZ);
    CHECK(data().com_position == p0);
    CHECK(data().com_velocity == kVec3DZero);
    CHECK(data().com_acceleration == kVec3DZero);
    CHECK(data().zmp_position == kVec3DZero);
    CHECK(data().reaction_force == Vec3D(0, 0, G));
    CHECK(data().external_force == kVec3DZero);
    CHECK(data().total_force == Vec3D(0, 0, G));
  }

  SECTION("with COM position and mass") {
    Fuzzer fuzz(0, 10);
    double m = fuzz.get();
    auto p0 = fuzz.get<Vec3D>();
    ComZmpModelData data(p0, m);
    CHECK(data().mass == m);
    CHECK(data().nu == kVec3DZ);
    CHECK(data().com_position == p0);
    CHECK(data().com_velocity == kVec3DZero);
    CHECK(data().com_acceleration == kVec3DZero);
    CHECK(data().zmp_position == kVec3DZero);
    CHECK(data().reaction_force == Vec3D(0, 0, m * G));
    CHECK(data().external_force == kVec3DZero);
    CHECK(data().total_force == Vec3D(0, 0, m * G));
  }
}

TEST_CASE("ComZmpModelData: copy constructor") {
  ComZmpModelData a;
  RandomizeData(a);
  ComZmpModelData b(a);
  CheckData(a, b);
  REQUIRE(a.get_ptr<0>() == b.get_ptr<0>());
}

TEST_CASE("ComZmpModelData: copy assignment operator") {
  ComZmpModelData a, b;
  RandomizeData(a);
  b = a;
  CheckData(a, b);
  REQUIRE(a.get_ptr<0>() == b.get_ptr<0>());
}

TEST_CASE("ComZmpModelData: move constructor") {
  double mass = 2.5;
  Vec3D com_pos = {2.0, 3.0, 4.0};
  ComZmpModelData a;
  auto ptr = a.get_ptr<0>();
  a().mass = mass;
  a().com_position = com_pos;

  ComZmpModelData b = std::move(a);
  CHECK(b().mass == mass);
  CHECK(b().com_position == com_pos);
  REQUIRE(b.get_ptr<0>() == ptr);

  auto f = [](ComZmpModelData arg) { return arg; };
  ComZmpModelData c = f(ComZmpModelData(com_pos, mass));
  CHECK(c().mass == mass);
}

TEST_CASE("ComZmpModelData: move assignment operator") {
  double mass = 2.5;
  Vec3D com_pos = {2.0, 3.0, 4.0};
  ComZmpModelData a, b, c;
  auto ptr = a.get_ptr<0>();
  a().mass = mass;
  a().com_position = com_pos;

  b = std::move(a);
  CHECK(b().mass == mass);
  CHECK(b().com_position == com_pos);
  REQUIRE(b.get_ptr<0>() == ptr);

  auto f = [](ComZmpModelData arg) { return arg; };
  c = f(ComZmpModelData(com_pos, mass));
  CHECK(c().mass == mass);
}

}  // namespace
}  // namespace holon
