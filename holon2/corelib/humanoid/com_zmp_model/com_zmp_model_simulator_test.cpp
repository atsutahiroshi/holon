/* com_zmp_model_simulator - Simple simulator of the dynamics of COM-ZMP model
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

#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_simulator.hpp"
#include "holon2/corelib/humanoid/com_zmp_model.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

const double kDefaultTimeStep = 0.001;
const Vec3d kDefaultComPosition = Vec3d(0, 0, 1);

void checkCtor(const ComZmpModelSimulator& sim, const Vec3d& expected_pg) {
  CHECK(sim.time() == 0.0);
  CHECK(sim.time_step() == kDefaultTimeStep);
  CHECK(sim.model().com_position() == expected_pg);
  CHECK(sim.getInitialComPosition() == expected_pg);
}
TEST_CASE("com_zmp_model_simulator: check c'tors",
          "[ComZmpModel][ComZmpModelSimulator]") {
  Random<Vec3d> rnd;
  SECTION("default c'tor") {
    ComZmpModelSimulator sim;
    checkCtor(sim, kDefaultComPosition);
  }
  SECTION("overloaded c'tor 1: with Data instance") {
    ComZmpModelData data;
    Vec3d v = rnd();
    data.get<1>().com_position = v;
    ComZmpModelSimulator sim(data);
    checkCtor(sim, v);
    CHECK(sim.model().data() == data);
  }
  SECTION("overloaded c'tor 2: with Model instance") {
    Vec3d v = rnd();
    auto model = ComZmpModelBuilder().setComPosition(v).build();
    ComZmpModelSimulator sim(model);
    checkCtor(sim, v);
    CHECK(sim.model().data() != model.data());
  }
}

TEST_CASE("com_zmp_model_simulator: set initial COM position",
          "[ComZmpModel][ComZmpModelSimulator]") {
  Random<Vec3d> rnd;
  ComZmpModelSimulator sim;
  Vec3d p0 = rnd();
  sim.setInitialComPosition(p0);
  CHECK(sim.getInitialComPosition() == p0);
}

TEST_CASE("com_zmp_model_simulator: ", "[ComZmpModel][ComZmpModelSimulator]") {}

}  // namespace
}  // namespace holon
