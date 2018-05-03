/* com_zmp_model - COM-ZMP model
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

#include "holon2/corelib/humanoid/com_zmp_model.hpp"

#include "holon2/corelib/common/random.hpp"
#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

void checkModel(const ComZmpModel& model, const double expected_mass,
                const double expected_vhp, const Vec3d& expected_pg) {
  // params
  CHECK(model.params().mass == expected_mass);
  CHECK(model.params().nu == kVec3dZ);
  CHECK(model.params().vhp == expected_vhp);
  // inputs
  Vec3d expected_pz(expected_pg[0], expected_pg[1], expected_vhp);
  Vec3d expected_f(0, 0, expected_mass * kGravAccel);
  CHECK(model.inputs().com_position == expected_pg);
  CHECK(model.inputs().com_velocity == kVec3dZero);
  CHECK(model.inputs().zmp_position == expected_pz);
  CHECK(model.inputs().reaction_force == expected_f);
  CHECK(model.inputs().external_force == kVec3dZero);
  // outputs
  CHECK(model.outputs().com_position == kVec3dZero);
  CHECK(model.outputs().com_velocity == kVec3dZero);
  CHECK(model.outputs().com_acceleration == kVec3dZero);
  CHECK(model.outputs().zmp_position == kVec3dZero);
  CHECK(model.outputs().reaction_force == kVec3dZero);
  CHECK(model.outputs().total_force == kVec3dZero);
}

TEST_CASE("com_zmp_model: check model builder (default)",
          "[ComZmpModelData][ComZmpModelBuilder]") {
  auto model = ComZmpModelBuilder().build();
  checkModel(model, 1.0, 0.0, {0, 0, 1});
}

TEST_CASE("com_zmp_model: check model builder (set parameters)",
          "[ComZmpModelData][ComZmpModelBuilder]") {
  Random<double> rnd(0, 1);
  auto m = rnd();
  auto vhp = rnd();
  Vec3d pg(rnd(), rnd(), rnd());
  auto model = ComZmpModelBuilder()
                   .setMass(m)
                   .setVirtualHorizontalPlane(vhp)
                   .setComPosition(pg)
                   .build();
  checkModel(model, m, vhp, pg);
}

TEST_CASE("com_zmp_model: check model builder (give external dataset)",
          "[ComZmpModelData][ComZmpModelBuilder]") {
  Random<double> rnd(0, 1);
  auto m = rnd();
  auto vhp = rnd();
  Vec3d pg(rnd(), rnd(), rnd());
  ComZmpModelData data;
  auto model = ComZmpModelBuilder()
                   .setMass(m)
                   .setVirtualHorizontalPlane(vhp)
                   .setComPosition(pg)
                   .build(data);
  checkModel(model, m, vhp, pg);
}

}  // namespace
}  // namespace holon
