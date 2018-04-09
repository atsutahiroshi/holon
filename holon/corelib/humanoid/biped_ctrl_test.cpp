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

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

void CheckCtor_common(const BipedCtrl& ctrl) {
  REQUIRE(ctrl.model().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
  REQUIRE(ctrl.model().data().get_ptr<1>() == ctrl.data().get_ptr<1>());
  REQUIRE(ctrl.model().data().get_ptr<2>() == ctrl.data().get_ptr<2>());
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
  Fuzzer fuzz;
  Vec3D p = fuzz.get<Vec3D>(), v = fuzz.get<Vec3D>();
  Vec3D pl = fuzz.get<Vec3D>(), vl = fuzz.get<Vec3D>();
  Vec3D pr = fuzz.get<Vec3D>(), vr = fuzz.get<Vec3D>();
  model.states<0>().com_position = p;
  model.states<0>().com_velocity = v;
  model.states<1>().position = pl;
  model.states<1>().velocity = vl;
  model.states<2>().position = pr;
  model.states<2>().velocity = vr;
  BipedCtrl ctrl(model);
  CheckCtor_common(ctrl);
  CHECK(ctrl.states<0>().com_position == p);
  CHECK(ctrl.states<0>().com_velocity == v);
  CHECK(ctrl.states<1>().position == pl);
  CHECK(ctrl.states<1>().velocity == vl);
  CHECK(ctrl.states<2>().position == pr);
  CHECK(ctrl.states<2>().velocity == vr);
  REQUIRE(ctrl.data().get_ptr<0>() != model.data().get_ptr<0>());
  REQUIRE(ctrl.data().get_ptr<1>() != model.data().get_ptr<1>());
  REQUIRE(ctrl.data().get_ptr<2>() != model.data().get_ptr<2>());
}

TEST_CASE("Check c'tors of BipedCtrl") {
  SECTION("Default c'tor") { CheckCtor_0(); }
  SECTION("Overloaded c'tor 1") { CheckCtor_1(); }
  SECTION("Overloaded c'tor 2") { CheckCtor_2(); }
}

}  // namespace
}  // namespace holon
