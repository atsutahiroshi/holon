/* biped_model - Biped robot model
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

#include "holon/corelib/humanoid/biped_model.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

namespace data {

void CheckMembers(const BipedModelData& data,
                  const Vec3D& expected_trunk_position, double expected_mass,
                  const Vec3D& expected_left_foot_position,
                  const Vec3D& expected_right_foot_position) {
  // COM-ZMP model
  CHECK(data.trunk().mass == expected_mass);
  CHECK(data.trunk().com_position == expected_trunk_position);
  // left foot
  CHECK(data.left_foot().position == expected_left_foot_position);
  // right foot
  CHECK(data.right_foot().position == expected_right_foot_position);
}

void CheckCtor_0() {
  BipedModelData data;
  CheckMembers(data, kVec3DZ, 1.0, kVec3DZero, kVec3DZero);
}
void CheckCtor_1() {
  Fuzzer fuzz(-2, 2);
  Vec3D p0{fuzz(), fuzz(), Fuzzer(0, 2).get()};
  double mass = Fuzzer(0, 1).get();
  double dist = Fuzzer(0, 1).get();
  BipedModelData data(p0, mass, dist);
  CheckMembers(data, p0, mass, {p0.x(), p0.y() + 0.5 * dist, 0},
               {p0.x(), p0.y() - 0.5 * dist, 0});
}

void CheckCtor_2() {
  Fuzzer rn(-2, 2);
  Fuzzer rn_p(0, 1);
  Vec3D p0 = {rn(), rn(), rn_p()};
  double mass = rn_p();
  Vec3D lfp = {rn(), rn(), rn_p()};
  Vec3D rfp = {rn(), rn(), rn_p()};
  BipedModelData data(p0, mass, lfp, rfp);
  CheckMembers(data, p0, mass, lfp, rfp);
}

TEST_CASE("Check c'tors of BipedModelData", "[BipedModelData][ctor]") {
  SECTION("Default c'tor") { CheckCtor_0(); }
  SECTION("Overloaded c'tor 1") { CheckCtor_1(); }
  SECTION("Overloaded c'tor 2") { CheckCtor_2(); }
}

}  // namespace data

void CheckMembers(const BipedModel& model, const Vec3D& expected_trunk_position,
                  double expected_mass,
                  const Vec3D& expected_left_foot_position,
                  const Vec3D& expected_right_foot_position) {
  // Inherited from ModelBase class
  CHECK(model.time() == 0.0);
  CHECK(model.time_step() == BipedModel::default_time_step);
  // COM-ZMP model
  CHECK(&model.data().trunk() == &model.trunk().states());
  CHECK(model.data().trunk().mass == expected_mass);
  CHECK(model.data().trunk().com_position == expected_trunk_position);
  CHECK(model.trunk().initial_com_position() == expected_trunk_position);
  // Left foot
  CHECK(&model.data().left_foot() == &model.left_foot().states());
  CHECK(model.data().left_foot().position == expected_left_foot_position);
  CHECK(model.left_foot().initial_position() == expected_left_foot_position);
  // Right foot
  CHECK(&model.data().right_foot() == &model.right_foot().states());
  CHECK(model.data().right_foot().position == expected_right_foot_position);
  CHECK(model.right_foot().initial_position() == expected_right_foot_position);
}

void CheckConstructor_0() {
  BipedModel model;
  CheckMembers(model, ComZmpModelRawData::default_com_position,
               ComZmpModelRawData::default_mass, kVec3DZero, kVec3DZero);
}

void CheckConstructor_1() {
  Fuzzer fuzz(-2, 2);
  Vec3D p0{fuzz(), fuzz(), Fuzzer(0, 2).get()};
  double mass = Fuzzer(0, 1).get();
  double dist = Fuzzer(0, 1).get();
  BipedModel model(p0, mass, dist);
  CheckMembers(model, p0, mass, {p0.x(), p0.y() + 0.5 * dist, 0},
               {p0.x(), p0.y() - 0.5 * dist, 0});
}

void CheckConstructor_2() {
  Fuzzer rn(-2, 2);
  Fuzzer rn_p(0, 1);
  Vec3D p0 = {rn(), rn(), rn_p()};
  double mass = rn_p();
  Vec3D lfp = {rn(), rn(), rn_p()};
  Vec3D rfp = {rn(), rn(), rn_p()};
  BipedModel model(p0, mass, lfp, rfp);
  CheckMembers(model, p0, mass, lfp, rfp);
}

TEST_CASE("Constructor of BipdeModel", "[BipdeModel][ctor]") {
  SECTION("Default constructor") { CheckConstructor_0(); }
  SECTION("Overloaded constructor 1") { CheckConstructor_1(); }
  SECTION("Overloaded constructor 2") { CheckConstructor_2(); }
}

}  // namespace
}  // namespace holon
