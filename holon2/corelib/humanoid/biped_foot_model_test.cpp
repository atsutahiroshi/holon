/* biped_foot_model - Biped foot model
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

#include "holon2/corelib/humanoid/biped_foot_model.hpp"
#include "holon2/corelib/common/random.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

TEST_CASE("biped_foot_model: accessor to data", "[BipedFootModel]") {
  Random<Vec3d> rnd;
  Vec3d v = rnd();
  BipedFootModel model;
  model.data().get<1>().position = v;
  CHECK(model.data().get<1>().position == v);
}

TEST_CASE("biped_foot_model: accessor to parameters", "[BipedFootModel]") {
  Random<double> rnd;
  double m = rnd();
  BipedFootModel model;
  model.params().mass = m;
  CHECK(model.params().mass == m);
}

TEST_CASE("biped_foot_model: accessor to states", "[BipedFootModel]") {
  Random<Vec3d> rnd;
  Vec3d v = rnd();
  BipedFootModel model;
  model.states().position = v;
  CHECK(model.states().position == v);
}

TEST_CASE("biped_foot_model: accessor to mass", "[BipedFootModel]") {
  double m = Random<double>().get();
  BipedFootModel model;
  model.params().mass = m;
  CHECK(model.mass() == m);
}

TEST_CASE("biped_foot_model: accessor to position", "[BipedFootModel]") {
  Vec3d v = Random<Vec3d>().get();
  BipedFootModel model;
  model.states().position = v;
  CHECK(model.position() == v);
}

TEST_CASE("biped_foot_model: accessor to velocity", "[BipedFootModel]") {
  Vec3d v = Random<Vec3d>().get();
  BipedFootModel model;
  model.states().velocity = v;
  CHECK(model.velocity() == v);
}

TEST_CASE("biped_foot_model: accessor to acceleration", "[BipedFootModel]") {
  Vec3d v = Random<Vec3d>().get();
  BipedFootModel model;
  model.states().acceleration = v;
  CHECK(model.acceleration() == v);
}

TEST_CASE("biped_foot_model: accessor to force", "[BipedFootModel]") {
  Vec3d v = Random<Vec3d>().get();
  BipedFootModel model;
  model.states().force = v;
  CHECK(model.force() == v);
}

TEST_CASE("biped_foot_model: clone model instance", "[BipedFootModel]") {
  Random<double> m;
  Random<Vec3d> v;
  auto model = BipedFootModelBuilder()
                   .setPosition(v())
                   .setVelocity(v())
                   .setMass(m())
                   .build();
  auto cln = model.clone();
  CHECK(cln.mass() == model.mass());
  CHECK(cln.position() == model.position());
  CHECK(cln.velocity() == model.velocity());
  CHECK(cln.data() != model.data());
}

TEST_CASE("biped_foot_model: ", "[BipedFootModel]") {}

}  // namespace
}  // namespace holon
