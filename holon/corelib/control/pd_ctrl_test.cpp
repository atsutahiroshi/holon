/* pd_ctrl - Simple PD control class
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of the holon.
 *
 * The holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "holon/corelib/control/pd_ctrl.hpp"

#include <memory>
#include "holon/corelib/math/vec3d.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace experimental {
namespace {

template <typename T>
void CheckCtor_0() {
  PdCtrl<T> ctrl;
  REQUIRE(ctrl.model_data() == ctrl.model().data());
}
template <typename T>
void CheckCtor_1() {
  Fuzzer fuzz;
  PointMassModel<T> model;
  model.states().mass = Fuzzer(0, 10).get<double>();
  model.states().position = fuzz.get<T>();
  model.states().velocity = fuzz.get<T>();

  PdCtrl<T> ctrl(model);
  REQUIRE(ctrl.model_data() != model.data());
  CHECK(ctrl.states().mass == model.states().mass);
  CHECK(ctrl.states().position == model.states().position);
  CHECK(ctrl.model().initial_position() == model.states().position);
  CHECK(ctrl.states().velocity == model.states().velocity);
  CHECK(ctrl.refs().position == model.states().position);
  CHECK(ctrl.refs().velocity == model.states().velocity);
}
TEST_CASE("C'tor of PdCtrl", "[PdCtrl][ctor]") {
  SECTION("Default c'tor") {
    CheckCtor_0<double>();
    CheckCtor_0<Vec3D>();
  }
  SECTION("Overloaded c'tor") {
    CheckCtor_1<double>();
    CheckCtor_1<Vec3D>();
  }
}

template <typename T>
void CheckReset_1() {
  Fuzzer fuzz;
  PdCtrl<T> ctrl;
  ctrl.states().position = fuzz.get<T>();
  ctrl.states().velocity = fuzz.get<T>();
  ctrl.update();
  REQUIRE(ctrl.time() != 0.0);
  REQUIRE(ctrl.states().position != T{0});
  REQUIRE(ctrl.states().velocity != T{0});
  ctrl.reset();
  CHECK(ctrl.time() == 0.0);
  CHECK(ctrl.states().position == T{0});
  CHECK(ctrl.states().velocity == T{0});
}

template <typename T>
void CheckReset_2() {
  Fuzzer fuzz;
  PdCtrl<T> ctrl;
  ctrl.states().position = fuzz.get<T>();
  ctrl.states().velocity = fuzz.get<T>();
  auto p = fuzz.get<T>();
  ctrl.update();
  REQUIRE(ctrl.time() != 0.0);
  REQUIRE(ctrl.states().position != p);
  REQUIRE(ctrl.states().velocity != T{0});
  ctrl.reset(p);
  CHECK(ctrl.time() == 0.0);
  CHECK(ctrl.states().position == p);
  CHECK(ctrl.states().velocity == T{0});
}

TEST_CASE("Check reset in PdCtrl", "[PdCtrl][reset]") {
  SECTION("Overloaded function 1") {
    CheckReset_1<double>();
    CheckReset_1<Vec3D>();
  }
  SECTION("Overloaded function 2") {
    CheckReset_2<double>();
    CheckReset_2<Vec3D>();
  }
}

template <typename T>
void CheckOutputsAfterUpdate() {
  Fuzzer fuzz;
  PdCtrl<T> ctrl;
  ctrl.states().mass = 2;
  ctrl.refs().position = T{1};
  ctrl.refs().stiffness = T{10};
  ctrl.refs().damping = T{1};
  ctrl.outputs().position = fuzz.get<T>();
  ctrl.outputs().velocity = fuzz.get<T>();
  ctrl.outputs().acceleration = fuzz.get<T>();
  ctrl.outputs().force = fuzz.get<T>();

  REQUIRE(ctrl.update());
  CHECK(ctrl.outputs().position == ctrl.states().position);
  CHECK(ctrl.outputs().velocity == ctrl.states().velocity);
  CHECK(ctrl.outputs().acceleration == ctrl.states().acceleration);
  CHECK(ctrl.outputs().force == ctrl.states().force);

  REQUIRE(ctrl.update(0.02));
  CHECK(ctrl.time_step() == 0.02);
  CHECK(ctrl.outputs().position == ctrl.states().position);
  CHECK(ctrl.outputs().velocity == ctrl.states().velocity);
  CHECK(ctrl.outputs().acceleration == ctrl.states().acceleration);
  CHECK(ctrl.outputs().force == ctrl.states().force);
}

TEST_CASE("Outputs should be updated after calling update of PdCtrl",
          "[PdCtrl][update]") {
  CheckOutputsAfterUpdate<double>();
  CheckOutputsAfterUpdate<Vec3D>();
}

SCENARIO("Point mass is converged at the desired position with PD contrl",
         "[PdCtrl][update]") {
  GIVEN("Point mass is currently at 1, desired position is 0") {
    PointMassModel<double> model;
    model.states().position = 1;
    PdCtrl<double> ctrl(model);
    REQUIRE(ctrl.states().position == 1);
    ctrl.refs().position = 0;
    ctrl.refs().stiffness = 100;
    ctrl.refs().damping = 10;
    WHEN("Update until 10 sec") {
      while (ctrl.time() < 10) {
        ctrl.update();
      }
      THEN("The point mass should be at 1") {
        INFO(ctrl.states().position);
        CHECK(ctrl.states().position == Approx(0.0).margin(1.0e-5));
      }
    }
  }
}

}  // namespace
}  // namespace experimental

#if 0
namespace {

template <typename T>
using Ctrl = PdCtrl<T>;

template <typename T>
using Model = PointMassModel<T>;

template <typename T>
using Refs = PdCtrlRefs<T>;

template <typename T>
using Outputs = PdCtrlOutputs<T>;

template <typename T>
void CheckConstructor_0() {
  Ctrl<T> ctrl;
  REQUIRE(ctrl.states_ptr() != nullptr);
  REQUIRE(ctrl.refs_ptr() != nullptr);
  REQUIRE(ctrl.outputs_ptr() != nullptr);
  REQUIRE(ctrl.model().data_ptr() == ctrl.states_ptr());
}

template <typename T>
void CheckConstructor_1() {
  Model<T> model;
  model.data_ptr()->mass = Fuzzer(0, 1).get<double>();
  model.data_ptr()->position = Fuzzer().get<T>();
  model.data_ptr()->velocity = Fuzzer().get<T>();

  Ctrl<T> ctrl(model);
  REQUIRE(ctrl.states_ptr() != model.data_ptr());
  CHECK(ctrl.model().mass() == model.data().mass);
  CHECK(ctrl.states().position == model.data().position);
  CHECK(ctrl.model().initial_position() == model.data().position);
  CHECK(ctrl.states().velocity == model.data().velocity);
  CHECK(ctrl.refs().position == model.data().position);
  CHECK(ctrl.refs().velocity == model.data().velocity);
  model.data_ptr()->position = Fuzzer().get<T>();
  CHECK(ctrl.states().position != model.data().position);
}

TEST_CASE("Constructor of PdCtrl", "[PdCtrl][ctor]") {
  SECTION("Default contructor") {
    CheckConstructor_0<double>();
    CheckConstructor_0<Vec3D>();
  }
  SECTION("Overloaded constructor 1") {
    CheckConstructor_1<double>();
    CheckConstructor_1<Vec3D>();
  }
}

template <typename T>
void CheckStatesPtr() {
  auto model = makePointMassModel<T>();
  Ctrl<T> ctrl;
  REQUIRE(ctrl.states_ptr() != model.data_ptr());
  ctrl.set_states_ptr(model.data_ptr());
  CHECK(ctrl.states_ptr() == model.data_ptr());
}

template <typename T>
void CheckStatesPtr2() {
  auto data = createPointMassModelData<T>();
  Ctrl<T> ctrl;
  REQUIRE(ctrl.states_ptr() != data);
  REQUIRE(ctrl.model().data_ptr() != data);
  ctrl.set_states_ptr(data);
  CHECK(ctrl.states_ptr() == data);
  CHECK(ctrl.model().data_ptr() == data);
}

template <typename T>
void CheckRefsPtr() {
  auto refs = std::make_shared<Refs<T>>();
  Ctrl<T> ctrl;
  REQUIRE(ctrl.refs_ptr() != refs);
  ctrl.set_refs_ptr(refs);
  CHECK(ctrl.refs_ptr() == refs);
}

template <typename T>
void CheckOutputsPtr() {
  auto outputs = std::make_shared<Outputs<T>>();
  Ctrl<T> ctrl;
  REQUIRE(ctrl.outputs_ptr() != outputs);
  ctrl.set_outputs_ptr(outputs);
  CHECK(ctrl.outputs_ptr() == outputs);
}

template <typename T>
void CheckTimeStep() {
  Ctrl<T> ctrl;
  double dt = Fuzzer(0, 0.01).get();
  REQUIRE(ctrl.time_step() != dt);
  ctrl.set_time_step(dt);
  CHECK(ctrl.time_step() == dt);
}

TEST_CASE("Accessors / mutators in PdCtrl", "[PdCtrl][accessor][mutator]") {
  SECTION("states pointer") {
    CheckStatesPtr<double>();
    CheckStatesPtr<Vec3D>();
  }
  SECTION("states pointer 2") {
    CheckStatesPtr2<double>();
    CheckStatesPtr2<Vec3D>();
  }
  SECTION("refs pointer") {
    CheckRefsPtr<double>();
    CheckRefsPtr<Vec3D>();
  }
  SECTION("outputs pointer") {
    CheckOutputsPtr<double>();
    CheckOutputsPtr<Vec3D>();
  }
  SECTION("time step") {
    CheckTimeStep<double>();
    CheckTimeStep<Vec3D>();
  }
}

template <typename T>
void CheckReset_1() {
  Ctrl<T> ctrl;
  Fuzzer fuzz;
  ctrl.states_ptr()->position = fuzz.get<T>();
  ctrl.states_ptr()->velocity = fuzz.get<T>();
  ctrl.update();
  REQUIRE(ctrl.time() != 0.0);
  REQUIRE(ctrl.states().position != T{0});
  REQUIRE(ctrl.states().velocity != T{0});
  ctrl.reset();
  CHECK(ctrl.time() == 0.0);
  CHECK(ctrl.states().position == T{0});
  CHECK(ctrl.states().velocity == T{0});
}

template <typename T>
void CheckReset_2() {
  Ctrl<T> ctrl;
  Fuzzer fuzz;
  ctrl.states_ptr()->position = fuzz.get<T>();
  ctrl.states_ptr()->velocity = fuzz.get<T>();
  auto p = fuzz.get<T>();
  ctrl.update();
  REQUIRE(ctrl.time() != 0.0);
  REQUIRE(ctrl.states().position != p);
  REQUIRE(ctrl.states().velocity != T{0});
  ctrl.reset(p);
  CHECK(ctrl.time() == 0.0);
  CHECK(ctrl.states().position == p);
  CHECK(ctrl.states().velocity == T{0});
}

TEST_CASE("Check reset in PdCtrl", "[PdCtrl][reset]") {
  SECTION("Overloaded function 1") {
    CheckReset_1<double>();
    CheckReset_1<Vec3D>();
  }
  SECTION("Overloaded function 2") {
    CheckReset_2<double>();
    CheckReset_2<Vec3D>();
  }
}

template <typename T>
void CheckOutputsAfterUpdate() {
  Ctrl<T> ctrl;
  ctrl.states_ptr()->mass = 2;
  ctrl.refs_ptr()->position = T{1};
  ctrl.refs_ptr()->stiffness = T{10};
  ctrl.refs_ptr()->damping = T{1};
  REQUIRE(ctrl.update());
  CHECK(ctrl.outputs().position == ctrl.states().position);
  CHECK(ctrl.outputs().velocity == ctrl.states().velocity);
  CHECK(ctrl.outputs().acceleration == ctrl.states().acceleration);
  CHECK(ctrl.outputs().force == ctrl.states().force);
}

TEST_CASE("Outputs should be updated after calling update of PdCtrl",
          "[PdCtrl][update]") {
  CheckOutputsAfterUpdate<double>();
  CheckOutputsAfterUpdate<Vec3D>();
}

SCENARIO("Point mass is converged at the desired position with PD contrl",
         "[PdCtrl][update]") {
  GIVEN("Point mass is currently at 1, desired position is 0") {
    PointMassModel<double> model;
    model.data_ptr()->position = 1;
    PdCtrl<double> ctrl(model);
    ctrl.refs_ptr()->position = 0;
    ctrl.refs_ptr()->stiffness = 100;
    ctrl.refs_ptr()->damping = 10;
    WHEN("Update until 10 sec") {
      while (ctrl.time() < 10) {
        ctrl.update();
      }
      THEN("The point mass should be at 1") {
        CHECK(ctrl.states().position == Approx(0.0).margin(1.0e-5));
      }
    }
  }
}

}  // namespace
#endif
}  // namespace holon
