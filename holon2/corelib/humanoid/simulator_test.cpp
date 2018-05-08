/* simulator - Base class of simulator
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

#include "holon2/corelib/humanoid/simulator.hpp"

#include "holon2/corelib/common/random.hpp"
#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

class SimulatorTest : public Simulator {
 public:
  virtual ~SimulatorTest() = default;
  void reset() final { return resetTime(); }
  bool update() final { return update(time_step()); }
  bool update(double dt) final {
    updateTime(dt);
    return true;
  }
};

const double kDefaultTimeStep = 0.001;

void checkCtor(const SimulatorTest& sim) {
  CHECK(sim.time() == 0.0);
  CHECK(sim.time_step() == kDefaultTimeStep);
}
TEST_CASE("simulator: check c'tors", "[Simulator]") {
  SECTION("default c'tor") {
    SimulatorTest sim;
    checkCtor(sim);
  }
}

TEST_CASE("simulator: set time step", "[Simulator]") {
  SimulatorTest sim;
  REQUIRE(sim.time_step() == kDefaultTimeStep);
  auto dt = Random<double>(0, 0.01).get();
  sim.setTimeStep(dt);
  CHECK(sim.time_step() == dt);
}

TEST_CASE("simulator: update current time", "[Simulator]") {
  SimulatorTest sim;
  Random<double> rnd(0, 0.01);
  SECTION("update with default time step") {
    REQUIRE(sim.time() == 0.0);
    sim.update();
    CHECK(sim.time() == kDefaultTimeStep);
    sim.update();
    CHECK(sim.time() == 2 * kDefaultTimeStep);
  }
  SECTION("modify time step") {
    auto dt = rnd();
    REQUIRE(sim.time() == 0.0);
    sim.setTimeStep(dt);
    sim.update();
    CHECK(sim.time() == dt);
    sim.update();
    CHECK(sim.time() == 2 * dt);
  }
  SECTION("modify time step temporariliy") {
    auto dt1 = rnd();
    REQUIRE(sim.time() == 0.0);
    sim.update(dt1);
    CHECK(sim.time() == dt1);
    CHECK(sim.time_step() == kDefaultTimeStep);
    auto dt2 = rnd();
    sim.update(dt2);
    CHECK(sim.time() == dt1 + dt2);
    CHECK(sim.time_step() == kDefaultTimeStep);
  }
}

TEST_CASE("simulator: reset time", "[Simulator]") {
  SimulatorTest sim;
  sim.update();
  REQUIRE(sim.time() != 0.0);
  sim.reset();
  CHECK(sim.time() == 0.0);
}

TEST_CASE("simulator: ", "[Simulator]") {}

}  // namespace
}  // namespace holon
