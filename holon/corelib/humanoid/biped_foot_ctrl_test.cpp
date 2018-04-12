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
  CHECK(data.get<1>().position == expected_p0);
  CHECK(data.get<1>().velocity == kVec3DZero);
  CHECK(data.get<1>().stiffness == kVec3DZero);
  CHECK(data.get<1>().damping == kVec3DZero);
  CHECK(data.get<1>().max_height == 0);
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

}  // namespace
}  // namespace holon
