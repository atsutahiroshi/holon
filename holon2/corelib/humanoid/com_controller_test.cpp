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
    CHECK(ctrl.data().subdata<0, 1>() == ctrl.model().data());
    CHECK(ctrl.data().subdata<0, 1>() != model.data());
    CHECK(ctrl.model().mass() == model.mass());
    CHECK(ctrl.model().com_position() == model.com_position());
  }
}

TEST_CASE("com_controller: accessor to model parameters", "[ComController]") {
  ComControllerData data;
  auto& params = data.get<0>();
  Random<double> rnd;
  params.mass = rnd();
  params.vhp = rnd();
  ComController ctrl(data);
  CHECK(ctrl.mass() == params.mass);
  CHECK(ctrl.vhp() == params.vhp);
}

TEST_CASE("com_controller: accessor to model states", "[ComController]") {
  ComControllerData data;
  auto& states = data.get<1>();
  Random<Vec3d> rnd;
  states.com_position = rnd();
  states.com_velocity = rnd();
  states.zmp_position = rnd();
  states.reaction_force = rnd();
  states.total_force = rnd();
  ComController ctrl(data);
  CHECK(ctrl.states().com_position == states.com_position);
  CHECK(ctrl.states().com_velocity == states.com_velocity);
  CHECK(ctrl.states().zmp_position == states.zmp_position);
  CHECK(ctrl.states().reaction_force == states.reaction_force);
  CHECK(ctrl.states().total_force == states.total_force);
}

TEST_CASE("com_controller: accessor to control parameters", "[ComController]") {
  ComControllerData data;
  auto& params = data.get<2>();
  Random<double> rnd;
  params.q1 = {rnd(), rnd(), rnd()};
  params.q2 = {rnd(), rnd(), rnd()};
  ComController ctrl(data);
  for (auto i = 0; i < 3; ++i) {
    CHECK(ctrl.params().q1[i] == params.q1[i]);
    CHECK(ctrl.params().q2[i] == params.q2[i]);
  }
}

TEST_CASE("com_controller: accessor to control outputs", "[ComController]") {
  ComControllerData data;
  auto& outputs = data.get<3>();
  Random<Vec3d> rnd;
  outputs.com_position = rnd();
  outputs.com_velocity = rnd();
  outputs.zmp_position = rnd();
  outputs.reaction_force = rnd();
  ComController ctrl(data);
  CHECK(ctrl.outputs().com_position == outputs.com_position);
  CHECK(ctrl.outputs().com_velocity == outputs.com_velocity);
  CHECK(ctrl.outputs().zmp_position == outputs.zmp_position);
  CHECK(ctrl.outputs().reaction_force == outputs.reaction_force);
}

TEST_CASE("com_controller: ", "[ComController]") {}

}  // namespace
}  // namespace holon
