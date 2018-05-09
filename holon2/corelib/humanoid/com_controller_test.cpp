/* com_controller - COM controller class
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

#include "holon2/corelib/humanoid/com_controller.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

ComZmpModel getRandomModel() {
  double mass = Random<double>(0, 2).get();
  double vhp = Random<double>(0, 1).get();
  Vec3d p0 = Random<Vec3d>(-1, 1).get();
  p0[2] += 2;  // make COM height be larger than vhp
  ComZmpModelBuilder b;
  b.setMass(mass).setVirtualHorizontalPlane(vhp).setComPosition(p0);
  return b.build();
}
TEST_CASE("com_controller: check c'tors", "[ComController]") {
  SECTION("default c'tor") {
    ComController ctrl;
    CHECK(ctrl.data().subdata<0, 1>() == ctrl.model().data());
  }
  SECTION("overloaded c'tor: with Data") {
    ComControllerData data;
    ComController ctrl(data);
    CHECK(ctrl.data() == data);
    CHECK(ctrl.data().subdata<0, 1>() == ctrl.model().data());
  }
  SECTION("overloaded c'tor: with Model") {
    auto model = getRandomModel();
    ComController ctrl(model);
    CHECK(ctrl.data().subdata<0, 1>() != model.data());
    CHECK(ctrl.model().mass() == model.mass());
    CHECK(ctrl.model().com_position() == model.com_position());
  }
}

TEST_CASE("com_controller: ", "[ComController]") {}

}  // namespace
}  // namespace holon
