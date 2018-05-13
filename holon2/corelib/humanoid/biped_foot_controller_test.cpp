/* biped_foot_controller - Biped foot controller class
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

#include "holon2/corelib/humanoid/biped_foot_controller.hpp"

#include "holon2/corelib/common/random.hpp"
#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

const double kDefaultTimeStep = 0.001;
const double kTOL = 1e-10;
using ::Catch::Matchers::ApproxEquals;

BipedFootModel getRandomModel() {
  double mass = Random<double>(0, 2).get();
  Vec3d p0 = Random<Vec3d>(-1, 1).get();
  BipedFootModelBuilder builder;
  builder.setMass(mass).setPosition(p0);
  return builder.build();
}

void checkCtor(const BipedFootController& ctrl, const double expected_mass,
               const Vec3d& expected_position) {
  CHECK(ctrl.data().subdata<0, 1>() == ctrl.model().data());
  CHECK(ctrl.mass() == expected_mass);
  CHECK(ctrl.params().position == expected_position);
  CHECK(ctrl.params().velocity == kVec3dZero);
  for (auto i = 0; i < 3; ++i) {
    CHECK(ctrl.params().stiffness[i] ==
          BipedFootController::default_stiffness[i]);
    CHECK(ctrl.params().damping[i] == BipedFootController::default_damping[i]);
  }
  CHECK(ctrl.params().max_height == BipedFootController::default_max_height);
  CHECK(ctrl.getInitialPosition() == expected_position);
}

TEST_CASE("biped_foot_controller: check c'tors", "[BipedFootController]") {
  using Builder = BipedFootModelBuilder;
  SECTION("default c'tor") {
    BipedFootController ctrl;
    checkCtor(ctrl, Builder::default_mass, Builder::default_position);
  }
  SECTION("overloaded c'tor: with Data") {
    BipedFootControllerData data;
    BipedFootController ctrl(data);
    checkCtor(ctrl, Builder::default_mass, Builder::default_position);
    CHECK(ctrl.data() == data);
  }
  SECTION("overloaded c'tor: with Model") {
    const auto model = getRandomModel();
    BipedFootController ctrl(model);
    checkCtor(ctrl, model.mass(), model.position());
    CHECK(ctrl.data().subdata<0, 1>() != model.data());
  }
}

TEST_CASE("biped_foot_controller: accessor to model parameters",
          "[BipedFootController]") {
  BipedFootControllerData data;
  auto& params = data.get<0>();
  Random<double> rnd;
  params.mass = rnd();
  BipedFootController ctrl(data);
  CHECK(ctrl.mass() == params.mass);
}

TEST_CASE("biped_foot_controller: accessor to model states",
          "[BipedFootController]") {
  BipedFootControllerData data;
  auto& states = data.get<1>();
  Random<Vec3d> rnd;
  states.position = rnd();
  states.velocity = rnd();
  states.acceleration = rnd();
  states.force = rnd();
  const BipedFootController ctrl(data);
  CHECK(ctrl.states().position == states.position);
  CHECK(ctrl.states().velocity == states.velocity);
  CHECK(ctrl.states().acceleration == states.acceleration);
  CHECK(ctrl.states().force == states.force);
}

TEST_CASE("biped_foot_controller: accessor to control parameters",
          "[BipedFootController]") {
  BipedFootControllerData data;
  auto& params = data.get<2>();
  Random<double> rnd;
  Random<Array3d> arr;
  params.stiffness = arr();
  params.damping = arr();
  params.max_height = rnd();
  const BipedFootController ctrl(data);
  for (auto i = 0; i < 3; ++i) {
    CHECK(ctrl.params().stiffness[i] == params.stiffness[i]);
    CHECK(ctrl.params().damping[i] == params.damping[i]);
  }
  CHECK(ctrl.params().max_height == params.max_height);
}

TEST_CASE("biped_foot_controller: accessor to control outputs",
          "[BipedFootController]") {
  BipedFootControllerData data;
  auto& outputs = data.get<3>();
  Random<Vec3d> rnd;
  outputs.position = rnd();
  outputs.velocity = rnd();
  outputs.acceleration = rnd();
  outputs.force = rnd();
  const BipedFootController ctrl(data);
  CHECK(ctrl.outputs().position == outputs.position);
  CHECK(ctrl.outputs().velocity == outputs.velocity);
  CHECK(ctrl.outputs().acceleration == outputs.acceleration);
  CHECK(ctrl.outputs().force == outputs.force);
}

TEST_CASE("biped_foot_controller: ", "[BipedFootController]") {}

}  // namespace
}  // namespace holon