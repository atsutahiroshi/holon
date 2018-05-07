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

TEST_CASE("com_zmp_model: accessor to data", "[ComZmpModel]") {
  Random<Vec3d> rnd;
  Vec3d v = rnd();
  ComZmpModel model;
  model.data().get<1>().com_position = v;
  CHECK(model.data().get<1>().com_position == v);
}

TEST_CASE("com_zmp_model: accessor to parameters", "[ComZmpModel]") {
  Random<double> rnd;
  double m = rnd();
  ComZmpModel model;
  model.params().mass = m;
  CHECK(model.params().mass == m);
}

TEST_CASE("com_zmp_model: accessor to states", "[ComZmpModel]") {
  Random<Vec3d> rnd;
  Vec3d v = rnd();
  ComZmpModel model;
  model.states().com_position = v;
  CHECK(model.states().com_position == v);
}

TEST_CASE("com_zmp_model: accessor to mass", "[ComZmpModel]") {
  double m = Random<double>().get();
  ComZmpModel model;
  model.params().mass = m;
  CHECK(model.mass() == m);
}

TEST_CASE("com_zmp_model: accessor to COM position", "[ComZmpModel]") {
  Vec3d v = Random<Vec3d>().get();
  ComZmpModel model;
  model.states().com_position = v;
  CHECK(model.com_position() == v);
}

TEST_CASE("com_zmp_model: accessor to COM velocity", "[ComZmpModel]") {
  Vec3d v = Random<Vec3d>().get();
  ComZmpModel model;
  model.states().com_velocity = v;
  CHECK(model.com_velocity() == v);
}

TEST_CASE("com_zmp_model: accessor to COM acceleration", "[ComZmpModel]") {
  Vec3d v = Random<Vec3d>().get();
  ComZmpModel model;
  model.states().com_acceleration = v;
  CHECK(model.com_acceleration() == v);
}

TEST_CASE("com_zmp_model: accessor to ZMP position", "[ComZmpModel]") {
  Vec3d v = Random<Vec3d>().get();
  ComZmpModel model;
  model.states().zmp_position = v;
  CHECK(model.zmp_position() == v);
}

TEST_CASE("com_zmp_model: accessor to reaction force", "[ComZmpModel]") {
  Vec3d v = Random<Vec3d>().get();
  ComZmpModel model;
  model.states().reaction_force = v;
  CHECK(model.reaction_force() == v);
}

TEST_CASE("com_zmp_model: accessor to external force", "[ComZmpModel]") {
  Vec3d v = Random<Vec3d>().get();
  ComZmpModel model;
  model.states().external_force = v;
  CHECK(model.external_force() == v);
}

TEST_CASE("com_zmp_model: accessor to total force", "[ComZmpModel]") {
  Vec3d v = Random<Vec3d>().get();
  ComZmpModel model;
  model.states().total_force = v;
  CHECK(model.total_force() == v);
}

}  // namespace
}  // namespace holon
