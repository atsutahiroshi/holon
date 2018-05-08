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

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

const double kDefaultTimeStep = 0.001;

TEST_CASE("com_zmp_model_simulator: check c'tors",
          "[ComZmpModel][ComZmpModelSimulator]") {
  SECTION("default c'tor") {
    ComZmpModelSimulator sim;
    // CHECK(sim.getInitialComPosition() == sim.model().com_position());
    CHECK(sim.time() == 0.0);
    CHECK(sim.time_step() == kDefaultTimeStep);
  }
  SECTION("overloaded c'tor 1") {}
  SECTION("overloaded c'tor 2") {}
}

}  // namespace
}  // namespace holon
