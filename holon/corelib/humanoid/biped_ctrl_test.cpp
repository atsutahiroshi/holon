/* biped_ctrl - Biped robot control
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

#include "holon/corelib/humanoid/biped_ctrl.hpp"

#include <memory>

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

void RandomizeModelData(BipedModel* model) {
  Fuzzer fuzz;
  // COM-ZMP model
  model->states<0>().com_position = fuzz.get<Vec3D>();
  model->states<0>().com_velocity = fuzz.get<Vec3D>();
  // left foot
  model->states<1>().position = fuzz.get<Vec3D>();
  model->states<1>().velocity = fuzz.get<Vec3D>();
  // right foot
  model->states<2>().position = fuzz.get<Vec3D>();
  model->states<2>().velocity = fuzz.get<Vec3D>();
}
void CheckCtor_common(const BipedCtrl& ctrl) {
  REQUIRE(ctrl.model().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
  REQUIRE(ctrl.model().data().get_ptr<1>() == ctrl.data().get_ptr<1>());
  REQUIRE(ctrl.model().data().get_ptr<2>() == ctrl.data().get_ptr<2>());

  REQUIRE(ctrl.trunk().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
  REQUIRE(ctrl.trunk().data().get_ptr<1>() == ctrl.data().get_ptr<3>());
  REQUIRE(ctrl.trunk().data().get_ptr<2>() == ctrl.data().get_ptr<6>());
  REQUIRE(ctrl.trunk().data().get_ptr<3>() == ctrl.data().get_ptr<9>());

  REQUIRE(ctrl.left_foot().data().get_ptr<0>() == ctrl.data().get_ptr<1>());
  REQUIRE(ctrl.left_foot().data().get_ptr<1>() == ctrl.data().get_ptr<4>());
  REQUIRE(ctrl.left_foot().data().get_ptr<2>() == ctrl.data().get_ptr<7>());

  REQUIRE(ctrl.right_foot().data().get_ptr<0>() == ctrl.data().get_ptr<2>());
  REQUIRE(ctrl.right_foot().data().get_ptr<1>() == ctrl.data().get_ptr<5>());
  REQUIRE(ctrl.right_foot().data().get_ptr<2>() == ctrl.data().get_ptr<8>());

  REQUIRE(ctrl.model().trunk_ptr() == ctrl.trunk().model_ptr());
  REQUIRE(ctrl.model().left_foot_ptr() == ctrl.left_foot().model_ptr());
  REQUIRE(ctrl.model().right_foot_ptr() == ctrl.right_foot().model_ptr());
}
void CheckCtor_0() {
  BipedCtrl ctrl;
  CheckCtor_common(ctrl);
}
void CheckCtor_1() {
  auto data = make_data<BipedCtrlData>();
  BipedCtrl ctrl(data);
  CheckCtor_common(ctrl);
  CHECK(ctrl.data().get_ptr<0>() == data.get_ptr<0>());
  CHECK(ctrl.data().get_ptr<1>() == data.get_ptr<1>());
  CHECK(ctrl.data().get_ptr<2>() == data.get_ptr<2>());
  CHECK(ctrl.data().get_ptr<3>() == data.get_ptr<3>());
  CHECK(ctrl.data().get_ptr<4>() == data.get_ptr<4>());
  CHECK(ctrl.data().get_ptr<5>() == data.get_ptr<5>());
  CHECK(ctrl.data().get_ptr<6>() == data.get_ptr<6>());
  CHECK(ctrl.data().get_ptr<7>() == data.get_ptr<7>());
  CHECK(ctrl.data().get_ptr<8>() == data.get_ptr<8>());
  CHECK(ctrl.data().get_ptr<9>() == data.get_ptr<9>());
}
void CheckCtor_2() {
  auto model = make_model<BipedModel>();
  RandomizeModelData(&model);
  BipedCtrl ctrl(model);
  CheckCtor_common(ctrl);
  CHECK(ctrl.states<0>().com_position == model.states<0>().com_position);
  CHECK(ctrl.states<0>().com_velocity == model.states<0>().com_velocity);
  CHECK(ctrl.states<1>().position == model.states<1>().position);
  CHECK(ctrl.states<1>().velocity == model.states<1>().velocity);
  CHECK(ctrl.states<2>().position == model.states<2>().position);
  CHECK(ctrl.states<2>().velocity == model.states<2>().velocity);
  REQUIRE(ctrl.data().get_ptr<0>() != model.data().get_ptr<0>());
  REQUIRE(ctrl.data().get_ptr<1>() != model.data().get_ptr<1>());
  REQUIRE(ctrl.data().get_ptr<2>() != model.data().get_ptr<2>());
}

void CheckCtor_3() {
  auto data = make_data<BipedCtrlData>();
  auto model_ptr = std::make_shared<BipedModel>();
  BipedCtrl ctrl(data, model_ptr);
  CheckCtor_common(ctrl);
  CHECK(ctrl.data().get_ptr<0>() == data.get_ptr<0>());
  CHECK(ctrl.data().get_ptr<1>() == data.get_ptr<1>());
  CHECK(ctrl.data().get_ptr<2>() == data.get_ptr<2>());
  CHECK(ctrl.data().get_ptr<3>() == data.get_ptr<3>());
  CHECK(ctrl.data().get_ptr<4>() == data.get_ptr<4>());
  CHECK(ctrl.data().get_ptr<5>() == data.get_ptr<5>());
  CHECK(ctrl.data().get_ptr<6>() == data.get_ptr<6>());
  CHECK(ctrl.data().get_ptr<7>() == data.get_ptr<7>());
  CHECK(ctrl.data().get_ptr<8>() == data.get_ptr<8>());
  CHECK(ctrl.data().get_ptr<9>() == data.get_ptr<9>());
  CHECK(&ctrl.model() == model_ptr.get());
  CHECK(ctrl.model().trunk_ptr() == model_ptr->trunk_ptr());
  CHECK(ctrl.model().left_foot_ptr() == model_ptr->left_foot_ptr());
  CHECK(ctrl.model().right_foot_ptr() == model_ptr->right_foot_ptr());
  CHECK(ctrl.data().get_ptr<0>() == model_ptr->data().get_ptr<0>());
  CHECK(ctrl.data().get_ptr<1>() == model_ptr->data().get_ptr<1>());
  CHECK(ctrl.data().get_ptr<2>() == model_ptr->data().get_ptr<2>());
}

TEST_CASE("Check c'tors of BipedCtrl") {
  SECTION("Default c'tor") { CheckCtor_0(); }
  SECTION("Overloaded c'tor 1") { CheckCtor_1(); }
  SECTION("Overloaded c'tor 2") { CheckCtor_2(); }
  SECTION("Overloaded c'tor 3") { CheckCtor_3(); }
}

void CheckReset_common(const BipedCtrl& ctrl, const Vec3D& expected_com_pos,
                       const Vec3D& expected_lf_pos,
                       const Vec3D& expected_rf_pos) {
  auto expected_dist = (expected_lf_pos - expected_rf_pos).norm();
  CHECK(ctrl.time() == 0.0);
  CHECK(ctrl.states<0>().com_position == expected_com_pos);
  CHECK(ctrl.states<0>().com_velocity == kVec3DZero);
  CHECK(ctrl.states<1>().position == expected_lf_pos);
  CHECK(ctrl.states<1>().velocity == kVec3DZero);
  CHECK(ctrl.states<2>().position == expected_rf_pos);
  CHECK(ctrl.states<2>().velocity == kVec3DZero);

  CHECK(ctrl.trunk().model().initial_com_position() == expected_com_pos);
  CHECK(ctrl.trunk().canonical_foot_dist() == Approx(expected_dist));
  CHECK(ctrl.params<0>().dist == Approx(expected_dist));
  CHECK(ctrl.left_foot().model().initial_position() == expected_lf_pos);
  CHECK(ctrl.right_foot().model().initial_position() == expected_rf_pos);
}

void CheckReset_1() {
  BipedCtrl ctrl;
  ctrl.update();
  RandomizeModelData(&ctrl.model());
  REQUIRE(ctrl.time() != 0.0);
  ctrl.reset();
  CheckReset_common(ctrl, {0, 0, 1}, kVec3DZero, kVec3DZero);
}
void CheckReset_2() {
  BipedCtrl ctrl;
  ctrl.update();
  RandomizeModelData(&ctrl.model());
  REQUIRE(ctrl.time() != 0.0);
  Fuzzer fuzz;
  auto p0 = fuzz.get<Vec3D>();
  auto dist = Fuzzer(0, 1).get();
  auto pl = Vec3D{p0.x(), p0.y() + 0.5 * dist, 0};
  auto pr = Vec3D{p0.x(), p0.y() - 0.5 * dist, 0};
  ctrl.reset(p0, dist);
  CheckReset_common(ctrl, p0, pl, pr);
}
void CheckReset_3() {
  BipedCtrl ctrl;
  ctrl.update();
  RandomizeModelData(&ctrl.model());
  REQUIRE(ctrl.time() != 0.0);
  Fuzzer fuzz;
  auto p0 = fuzz.get<Vec3D>();
  auto pl = fuzz.get<Vec3D>();
  auto pr = fuzz.get<Vec3D>();
  auto dist = (pl - pr).norm();
  ctrl.reset(p0, pl, pr);
  CheckReset_common(ctrl, p0, pl, pr);
}

TEST_CASE("Check reset of BipedCtrl") {
  SECTION("Overloaded function 1") { CheckReset_1(); }
  SECTION("Overloaded function 2") { CheckReset_2(); }
  SECTION("Overloaded function 2") { CheckReset_3(); }
}

void CheckTimeUpdate_1() {
  BipedCtrl ctrl;
  auto dt = ComZmpModel::default_time_step;
  CHECK(ctrl.time() == 0.0);
  CHECK(ctrl.time_step() == Approx(dt));
  ctrl.update();
  CHECK(ctrl.time() == Approx(dt));
  CHECK(ctrl.time_step() == Approx(dt));
  ctrl.update();
  CHECK(ctrl.time() == Approx(2.0 * dt));
  CHECK(ctrl.time_step() == Approx(dt));
}

void CheckTimeUpdate_2() {
  Fuzzer fuzz(0, 0.01);
  BipedCtrl ctrl;
  CHECK(ctrl.time() == 0.0);
  CHECK(ctrl.time_step() == Approx(ComZmpModel::default_time_step));
  auto dt1 = fuzz();
  ctrl.update(dt1);
  CHECK(ctrl.time() == Approx(dt1));
  CHECK(ctrl.time_step() == Approx(dt1));
  auto dt2 = fuzz();
  ctrl.update(dt2);
  CHECK(ctrl.time() == Approx(dt1 + dt2));
  CHECK(ctrl.time_step() == Approx(dt2));
}

TEST_CASE("Check time update of BipedCtrl", "[BipedCtrl][update]") {
  SECTION("Overloaded function 1") { CheckTimeUpdate_1(); }
  SECTION("Overloaded function 2") { CheckTimeUpdate_2(); }
}

}  // namespace
}  // namespace holon
