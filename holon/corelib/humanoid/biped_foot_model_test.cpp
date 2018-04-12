/* biped_foot_model - Bipedal foot model
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

#include "holon/corelib/humanoid/biped_foot_model.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

namespace data {

void CheckCtor_common(const BipedFootModelData& data,
                      const Vec3D& expected_p0) {
  CHECK(data.get().mass == 1.0);
  CHECK(data.get().position == expected_p0);
  CHECK(data.get().velocity == kVec3DZero);
  CHECK(data.get().acceleration == kVec3DZero);
  CHECK(data.get().force == kVec3DZero);
}
void CheckCtor_0() {
  BipedFootModelData data;
  CheckCtor_common(data, kVec3DZero);
}
void CheckCtor_1() {
  Fuzzer fuzz;
  auto p0 = fuzz.get<Vec3D>();
  BipedFootModelData data(p0);
  CheckCtor_common(data, p0);
}
TEST_CASE("Check c'tor @BipedFootModelData", "[BipedFootModelData][ctor]") {
  SECTION("Default c'tor") { CheckCtor_0(); }
  SECTION("Overloaded c'tor 1") { CheckCtor_1(); }
}

}  // namespace data

void CheckCtor_common(const BipedFootModel& model, const Vec3D& expected_p0) {
  CHECK(model.states().mass == 1.0);
  CHECK(model.states().position == expected_p0);
  CHECK(model.states().velocity == kVec3DZero);
  CHECK(model.states().acceleration == kVec3DZero);
  CHECK(model.states().force == kVec3DZero);
}
void CheckCtor_0() {
  BipedFootModel model;
  CheckCtor_common(model, kVec3DZero);
  CHECK(model.type() == BipedFootType::none);
}
void CheckCtor_1() {
  Fuzzer fuzz;
  auto p0 = fuzz.get<Vec3D>();
  BipedFootModel model(p0);
  CheckCtor_common(model, p0);
  CHECK(model.type() == BipedFootType::none);
}
void CheckCtor_2() {
  Fuzzer fuzz;
  Vec3D pl = fuzz.get<Vec3D>(), pr = fuzz.get<Vec3D>();

  // left foot
  BipedFootModel left_foot(pl, BipedFootType::left);
  CheckCtor_common(left_foot, pl);
  CHECK(left_foot.type() == BipedFootType::left);
  // right foot
  BipedFootModel right_foot(pr, BipedFootType::right);
  CheckCtor_common(right_foot, pr);
  CHECK(right_foot.type() == BipedFootType::right);
}
TEST_CASE("Check c'tor @BipedFootModel", "[BipedFootModel][ctor]") {
  SECTION("Default c'tor") { CheckCtor_0(); }
  SECTION("Overloaded c'tor 1") { CheckCtor_1(); }
  SECTION("Overloaded c'tor 2") { CheckCtor_2(); }
}

TEST_CASE("Check function to return direction of foot @BipedFootModel",
          "[BipedFootModel][ctor]") {
  SECTION("left foot") {
    BipedFootModel lf(BipedFootType::left);
    CHECK(lf.dir() == 1);
  }
  SECTION("right foot") {
    BipedFootModel rf(BipedFootType::right);
    CHECK(rf.dir() == -1);
  }
}

}  // namespace
}  // namespace holon
