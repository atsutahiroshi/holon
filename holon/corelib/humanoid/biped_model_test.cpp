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

namespace holon {
namespace {

void CheckMembers(const BipedModel& model, const Vec3D& expected_trunk_position,
                  double expected_mass,
                  const Vec3D& expected_left_foot_position,
                  const Vec3D& expected_right_foot_position) {
  // Inherited from ModelBase class
  CHECK(model.time() == 0.0);
  CHECK(model.time_step() == BipedModel::default_time_step);
  // COM-ZMP model
  // CHECK(model.data().trunk_ptr() == model.trunk().data_ptr());
  // CHECK(model.data().trunk().mass == expected_mass);
  // CHECK(model.data().trunk().com_position == expected_trunk_position);
  // CHECK(model.trunk().initial_com_position() == expected_trunk_position);
  // Left foot
  // CHECK(model.data().lfoot_ptr() == model.lfoot().data_ptr());
  // CHECK(model.data().lfoot().position == expected_left_foot_position);
  // CHECK(model.lfoot().initial_position() == expected_left_foot_position);
  // Right foot
  // CHECK(model.data().rfoot_ptr() == model.rfoot().data_ptr());
  // CHECK(model.data().rfoot().position == expected_right_foot_position);
  // CHECK(model.rfoot().initial_position() == expected_right_foot_position);
}

void CheckConstructor_0() {
  BipedModel model;
  CheckMembers(model, ComZmpModelData::default_com_position,
               ComZmpModelData::default_mass, kVec3DZero, kVec3DZero);
}

void CheckConstructor_1() {
  Vec3D p0 = {0.5, -0.5, 0.75};
  double mass = 2.0;
  double dist = 0.42;
  BipedModel model(p0, mass, dist);
  CheckMembers(model, p0, mass, {p0.x(), p0.y() + 0.5 * dist, 0},
               {p0.x(), p0.y() - 0.5 * dist, 0});
}

void CheckConstructor_2() {
  Vec3D p0 = {-0.1, 0.1, 0.5};
  double mass = 1.5;
  Vec3D lfp = {-0.15, 0.25, 0};
  Vec3D rfp = {-0.05, -0.05, 0};
  double dist = 0.3;
  BipedModel model(p0, mass, lfp, rfp);
  CheckMembers(model, p0, mass, lfp, rfp);
}

TEST_CASE("Constructor of BipdeModel", "[BipdeModel][ctor]") {
  // SECTION("Default constructor") { CheckConstructor_0(); }
  // SECTION("Overloaded constructor 1") { CheckConstructor_1(); }
}

}  // namespace
}  // namespace holon
