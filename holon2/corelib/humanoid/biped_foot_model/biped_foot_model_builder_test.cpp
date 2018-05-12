/* biped_foot_model_builder - Builder class of biped foot model
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of holon.
 *
 * Holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "holon2/corelib/humanoid/biped_foot_model/biped_foot_model_builder.hpp"
#include "holon2/corelib/humanoid/biped_foot_model.hpp"
#include "holon2/corelib/humanoid/const_defs.hpp"

#include "holon2/corelib/common/random.hpp"
#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

void checkModel(const BipedFootModel& model, const double expected_mass,
                const Vec3d& expected_p0, const Vec3d& expected_v0) {
  // params
  CHECK(model.mass() == expected_mass);
  // states
  Vec3d expected_f(0, 0, expected_mass * kGravAccel);
  CHECK(model.position() == expected_p0);
  CHECK(model.velocity() == expected_v0);
  CHECK(model.acceleration() == kVec3dZero);
  CHECK(model.force() == expected_f);
}

TEST_CASE("biped_foot_model_builder: check model builder (default)",
          "[BipedFootModel][BipedFootModelBuilder]") {
  auto model = BipedFootModelBuilder().build();
  checkModel(model, 1.0, {0, 0, 1}, {0, 0, 0});
}

TEST_CASE("biped_foot_model_builder: check model builder (set parameters)",
          "[BipedFootModel][BipedFootModelBuilder]") {
  Random<double> rnd(0, 1);
  Random<Vec3d> vec;
  const double m = rnd();
  const Vec3d p0 = vec();
  const Vec3d v0 = vec();
  auto model = BipedFootModelBuilder()
                   .setMass(m)
                   .setPosition(p0)
                   .setVelocity(v0)
                   .build();
  checkModel(model, m, p0, v0);
}

TEST_CASE("biped_foot_model_builder: check model builder (external dataset)",
          "[BipedFootModel][BipedFootModelBuilder]") {
  Random<double> rnd(0, 1);
  Random<Vec3d> vec;
  const double m = rnd();
  const Vec3d p0 = vec();
  const Vec3d v0 = vec();
  BipedFootModelData data;
  auto model =
      BipedFootModelBuilder().setMass(m).setPosition(p0).setVelocity(v0).build(
          data);
  checkModel(model, m, p0, v0);
}

TEST_CASE("biped_foot_model_builder: ",
          "[BipedFootModel][BipedFootModelBuilder]") {}

}  // namespace
}  // namespace holon
