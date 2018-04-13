/* biped_foot_ctrl - Bipedal foot control
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

#include "holon/corelib/humanoid/biped_foot_ctrl.hpp"

#include <memory>
#include <type_traits>

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

namespace data {

void CheckCtor_common(const BipedFootCtrlData& data, const Vec3D& expected_p0) {
  // model data
  CHECK(data.get<0>().mass == 1.0);
  CHECK(data.get<0>().position == expected_p0);
  CHECK(data.get<0>().velocity == kVec3DZero);
  CHECK(data.get<0>().acceleration == kVec3DZero);
  CHECK(data.get<0>().force == kVec3DZero);
  // params data
  using ParamsRawData = BipedFootCtrlParamsRawData;
  CHECK(data.get<1>().position == expected_p0);
  CHECK(data.get<1>().velocity == kVec3DZero);
  CHECK(data.get<1>().stiffness == ParamsRawData::default_stiffness);
  CHECK(data.get<1>().damping == ParamsRawData::default_damping);
  CHECK(data.get<1>().max_height == ParamsRawData::default_max_height);
  // outputs data
  CHECK(data.get<2>().position == kVec3DZero);
  CHECK(data.get<2>().velocity == kVec3DZero);
  CHECK(data.get<2>().acceleration == kVec3DZero);
  CHECK(data.get<2>().force == kVec3DZero);
  // commands data
  CHECK(data.get<3>().max_height == nullopt);
  CHECK(data.get<3>().stiffness[0] == nullopt);
  CHECK(data.get<3>().stiffness[1] == nullopt);
  CHECK(data.get<3>().stiffness[2] == nullopt);
  CHECK(data.get<3>().damping[0] == nullopt);
  CHECK(data.get<3>().damping[1] == nullopt);
  CHECK(data.get<3>().damping[2] == nullopt);
}
void CheckCtor_0() {
  BipedFootCtrlData data;
  CheckCtor_common(data, kVec3DZero);
}
void CheckCtor_1() {
  Fuzzer fuzz;
  auto p0 = fuzz.get<Vec3D>();
  BipedFootCtrlData data(p0);
  CheckCtor_common(data, p0);
}

TEST_CASE("Check c'tors @BipedFootCtrlData", "[BipedFootCtrlData][ctor]") {
  SECTION("Default c'tor") { CheckCtor_0(); }
  SECTION("Overloaded c'tor 1") { CheckCtor_1(); }
}

}  // namespace data

void RandomizeModelData(BipedFootModel* model) {
  Fuzzer fuzz;
  model->states().position = fuzz.get<Vec3D>();
  model->states().velocity = fuzz.get<Vec3D>();
  model->states().acceleration = fuzz.get<Vec3D>();
  model->states().force = fuzz.get<Vec3D>();
}
void CheckData(const BipedFootModelData& data1,
               const BipedFootModelData& data2) {
  CHECK(data1.get().position == data2.get().position);
  CHECK(data1.get().velocity == data2.get().velocity);
  CHECK(data1.get().acceleration == data2.get().acceleration);
  CHECK(data1.get().force == data2.get().force);
}
void CheckCtor_common(const BipedFootCtrl& ctrl) {
  REQUIRE(ctrl.model().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
}
void CheckCtor_0() {
  BipedFootCtrl ctrl;
  CheckCtor_common(ctrl);
}
void CheckCtor_1() {
  auto model = make_model<BipedFootModel>();
  RandomizeModelData(&model);
  BipedFootCtrl ctrl(model);
  CheckCtor_common(ctrl);
  CHECK(ctrl.data().get_ptr<0>() != model.data().get_ptr<0>());
  CheckData(ctrl.model().data(), model.data());
}
void CheckCtor_2() {
  auto data = make_data<BipedFootCtrlData>();
  BipedFootCtrl ctrl(data);
  CheckCtor_common(ctrl);
  CHECK(ctrl.data() == data);
}
void CheckCtor_3() {
  auto data = make_data<BipedFootCtrlData>();
  auto model_ptr = std::make_shared<BipedFootModel>();
  BipedFootCtrl ctrl(data, model_ptr);
  CheckCtor_common(ctrl);
  CHECK(ctrl.data() == data);
  CHECK(&ctrl.model() == model_ptr.get());
  CHECK(ctrl.data().get_ptr<0>() == model_ptr->data().get_ptr<0>());
}
TEST_CASE("Check c'tors @BipedFootCtrl") {
  SECTION("Default c'tor") { CheckCtor_0(); }
  SECTION("Overloaded c'tor 1") { CheckCtor_1(); }
  SECTION("Overloaded c'tor 2") { CheckCtor_2(); }
  SECTION("Overloaded c'tor 3") { CheckCtor_3(); }
}

void CheckReset_common(const BipedFootCtrl& ctrl,
                       const BipedFootCtrlData& expected_data) {
  CHECK(ctrl.time() == 0.0);
  // model
  CAPTURE(ctrl.data().get<0>().position);
  CHECK(ctrl.data().get<0>().position == expected_data.get<0>().position);
  CHECK(ctrl.data().get<0>().velocity == expected_data.get<0>().velocity);
  CHECK(ctrl.data().get<0>().acceleration ==
        expected_data.get<0>().acceleration);
  CHECK(ctrl.data().get<0>().force == expected_data.get<0>().force);
  CHECK(ctrl.initial_position() == expected_data.get<0>().position);
}
void CheckReset_1() {
  BipedFootCtrl ctrl;
  ctrl.update();
  RandomizeModelData(&ctrl.model());
  REQUIRE(ctrl.time() != 0.0);
  ctrl.reset();
  CheckReset_common(ctrl, BipedFootCtrlData(kVec3DZero));
}
void CheckReset_2() {
  BipedFootCtrl ctrl;
  ctrl.update();
  RandomizeModelData(&ctrl.model());
  REQUIRE(ctrl.time() != 0.0);
  auto p0 = Fuzzer().get<Vec3D>();
  ctrl.reset(p0);
  CheckReset_common(ctrl, BipedFootCtrlData(p0));
}
TEST_CASE("Check reset @BipedFootCtrl") {
  SECTION("Overloaded function 1") { CheckReset_1(); }
  SECTION("Overloaded function 2") { CheckReset_2(); }
}

void CheckTimeUpdate_common(const BipedFootCtrl& ctrl, double t, double dt) {
  CAPTURE(t);
  CAPTURE(dt);
  CHECK(ctrl.time() == Approx(t));
  CHECK(ctrl.time_step() == Approx(dt));
}
void CheckTimeUpdate_1() {
  BipedFootCtrl ctrl;
  auto dt = BipedFootModel::default_time_step;
  CheckTimeUpdate_common(ctrl, 0, dt);
  ctrl.update();
  CheckTimeUpdate_common(ctrl, dt, dt);
  ctrl.update();
  CheckTimeUpdate_common(ctrl, 2 * dt, dt);
}
void CheckTimeUpdate_2() {
  Fuzzer fuzz(0, 0.01);
  BipedFootCtrl ctrl;
  CheckTimeUpdate_common(ctrl, 0, BipedFootModel::default_time_step);
  auto dt1 = fuzz();
  ctrl.update(dt1);
  CheckTimeUpdate_common(ctrl, dt1, dt1);
  auto dt2 = fuzz();
  ctrl.update(dt2);
  CheckTimeUpdate_common(ctrl, dt1 + dt2, dt2);
}
TEST_CASE("Check time update @BipedFootCtrl", "[BipedFootCtrl][update]") {
  SECTION("Overloaded function 1") { CheckTimeUpdate_1(); }
  SECTION("Overloaded function 2") { CheckTimeUpdate_2(); }
}

}  // namespace
}  // namespace holon
