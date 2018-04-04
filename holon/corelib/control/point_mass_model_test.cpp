/* point_mass_model - Point mass model
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

#include "holon/corelib/control/point_mass_model.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

template <typename State>
using Data = PointMassModelData<State>;
template <typename State>
using Model = PointMassModel<State>;

template <typename T>
void CheckInitializedMembers(const Model<T>& model, const T& expected_position,
                             const double expected_mass) {
  CHECK(model.time() == 0.0);
  CHECK(model.time_step() == Model<T>::default_time_step);
  CHECK(model.states().position == expected_position);
  CHECK(model.states().mass == expected_mass);
  CHECK(model.mass() == expected_mass);
  CHECK(model.system().data() == model.data());
  CHECK(model.initial_position() == expected_position);
}

template <typename T>
void CheckConstructor_1() {
  Model<T> model;
  CheckInitializedMembers(model, T(0.0), 1.0);
}

template <typename T>
void CheckConstructor_2() {
  auto v = Fuzzer().get<T>();
  Model<T> model(v);
  CheckInitializedMembers(model, v, 1.0);
}

template <typename T>
void CheckConstructor_3() {
  auto v = Fuzzer().get<T>();
  double mass = Fuzzer(0, 10).get();
  Model<T> model(v, mass);
  CheckInitializedMembers(model, v, mass);
}

template <typename T>
void CheckConstructor_4() {
  auto v = Fuzzer().get<T>();
  double mass = Fuzzer(0, 10).get();
  auto data = make_data<Data>(v, mass);
  Model<T> model(data);
  CheckInitializedMembers(model, v, mass);
  CHECK(model.data() == data);
  CHECK(model.system().data() == data);
}

TEST_CASE("Constructor of PointMassModel", "[PointMassModel][ctor]") {
  SECTION("Default constructor") {
    CheckConstructor_1<double>();
    CheckConstructor_1<Vec3D>();
  }
  SECTION("Overloaded constructor with State") {
    CheckConstructor_2<double>();
    CheckConstructor_2<Vec3D>();
  }
  SECTION("Overloaded constructor with State, double") {
    CheckConstructor_3<double>();
    CheckConstructor_3<Vec3D>();
  }
  SECTION("Overloaded constructor with DataPtr") {
    CheckConstructor_4<double>();
    CheckConstructor_4<Vec3D>();
  }
}

template <typename T>
void CheckTimeStep() {
  double dt = Fuzzer(0, 0.01).get();
  auto model = make_model<Model<T>>();
  REQUIRE(model.time_step() != dt);
  model.set_time_step(dt);
  CHECK(model.time_step() == Approx(dt));
}

template <typename T>
void CheckDataPtr() {
  auto data = make_data<Data<T>>();
  auto model = make_model<Model<T>>();
  REQUIRE(model.data() != data);
  model.set_data(data);
  CHECK(model.data() == data);
}

template <typename T>
void CheckInitialPosition() {
  Fuzzer fuzz;
  auto p0 = fuzz.get<T>();
  auto p1 = fuzz.get<T>();
  auto model = make_model<Model<T>>(p0);
  REQUIRE(model.initial_position() == p0);
  REQUIRE(model.initial_position() != p1);
  model.set_initial_position(p1);
  CHECK(model.initial_position() == p1);
}

TEST_CASE("Check accssors / mutators in PointMassModel",
          "[PointMassModel][accessor][mutator]") {
  SECTION("time step") {
    CheckTimeStep<double>();
    CheckTimeStep<Vec3D>();
  }
  SECTION("data pointer") {
    CheckDataPtr<double>();
    CheckDataPtr<Vec3D>();
  }
  SECTION("initial position") {
    CheckInitialPosition<double>();
    CheckInitialPosition<Vec3D>();
  }
}

template <typename T>
void CheckReset_1() {
  Fuzzer fuzz;
  auto model = make_model<Model<T>>();
  model.states().position = fuzz.get<T>();
  model.states().velocity = fuzz.get<T>();
  model.update();
  REQUIRE(model.states().position != model.initial_position());
  REQUIRE(model.states().velocity != T{0});
  REQUIRE(model.time() != 0);
  model.reset();
  CHECK(model.states().position == model.initial_position());
  CHECK(model.states().velocity == T{0});
  CHECK(model.time() == 0);
}

template <typename T>
void CheckReset_2() {
  Fuzzer fuzz;
  auto model = make_model<Model<T>>();
  model.states().position = fuzz.get<T>();
  model.states().velocity = fuzz.get<T>();
  model.update();
  REQUIRE(model.states().position != model.initial_position());
  REQUIRE(model.states().velocity != T{0});
  REQUIRE(model.time() != 0);
  auto v = fuzz.get<T>();
  model.reset(v);
  CHECK(model.initial_position() == v);
  CHECK(model.states().position == model.initial_position());
  CHECK(model.states().velocity == T{0});
  CHECK(model.time() == 0);
}

TEST_CASE("Check reset function in PointMassModel", "[PointMassModel][reset]") {
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
void CheckSetForceCallback() {
  auto model = make_model<Model<T>>();
  auto force = Fuzzer().get<T>();
  auto func = [force](const T&, const T&, const double&) { return force; };
  model.setForceCallback(func);
  CHECK(model.system().force(T{0}, T{0}, 0) == force);
}

TEST_CASE("Check setForceCallback in PointMassModel",
          "[PointMassModel][setForceCallback]") {
  CheckSetForceCallback<double>();
  CheckSetForceCallback<Vec3D>();
}

template <typename T>
void CheckUpdateTime() {
  auto model = make_model<Model<T>>();
  auto dt = Model<T>::default_time_step;
  REQUIRE(model.time() == 0);
  REQUIRE(model.time_step() == Approx(dt));
  model.update();
  CHECK(model.time() == Approx(dt));
  model.update();
  CHECK(model.time() == Approx(dt + dt));
  auto dt2 = Fuzzer(0, 0.001).get();
  model.update(dt2);
  CHECK(model.time() == Approx(dt + dt + dt2));
}

TEST_CASE("Check if time is updated by update in PointMassModel",
          "[PointMassModel][update]") {
  CheckUpdateTime<double>();
  CheckUpdateTime<Vec3D>();
}

template <typename T>
void CheckUpdateWithoutForce() {
  auto model = make_model<Model<T>>();
  while (model.time() < 1) model.update();
  CHECK(model.states().position == T{0.0});
}

TEST_CASE("Check if point mass is still without adding force",
          "[PointMassModel][update]") {
  CheckUpdateWithoutForce<double>();
  CheckUpdateWithoutForce<Vec3D>();
}

TEST_CASE("Check if point mass is moving with adding force, one dimentinal",
          "[PointMassModel][update]") {
  auto model = make_model<Model<double>>();
  auto f = [](const double&, const double&, const double) { return 1.0; };
  model.setForceCallback(f);
  REQUIRE(model.states().position == 0);
  REQUIRE(model.time() == 0);
  while (model.time() < 0.1) model.update();
  CHECK(model.states().position > 0);
  CHECK(model.time() == Approx(0.1));
}

TEST_CASE("Check if point mass is moving with adding force",
          "[PointMassModel][update]") {
  Vec3D force = {1, -1, 0};
  auto model = make_model<Model<Vec3D>>();
  auto f = [force](const Vec3D&, const Vec3D&, const double) { return force; };
  model.setForceCallback(f);
  REQUIRE(model.states().position.x() == 0);
  REQUIRE(model.states().position.y() == 0);
  REQUIRE(model.states().position.z() == 0);
  REQUIRE(model.time() == 0);
  while (model.time() < 0.1) model.update();
  CHECK(model.states().position.x() > 0);
  CHECK(model.states().position.y() < 0);
  CHECK(model.states().position.z() == 0);
  CHECK(model.time() == Approx(0.1));
}

template <typename T>
void CheckDataAfterUpdate_checker(const Model<T>& model);

template <>
void CheckDataAfterUpdate_checker(const Model<double>& model) {
  CHECK(model.states().position > 0);
  CHECK(model.states().velocity > 0);
  CHECK(model.states().acceleration == Approx(0.5));
  CHECK(model.states().force == Approx(1.0));
}

template <>
void CheckDataAfterUpdate_checker(const Model<Vec3D>& model) {
  for (const auto& elem : model.states().position) {
    CHECK(elem > 0);
  }
  for (const auto& elem : model.states().velocity) {
    CHECK(elem > 0);
  }
  CHECK(model.states().acceleration == Vec3D(0.5));
  CHECK(model.states().force == Vec3D(1.0));
}

template <typename T>
void CheckDataAfterUpdate() {
  auto model = make_model<Model<T>>(T{0}, 2);
  auto func = [](const T&, const T&, const double) { return T{1}; };
  model.setForceCallback(func);
  REQUIRE(model.update());
  CheckDataAfterUpdate_checker(model);
}

TEST_CASE("Check if data is updated after update of PointMassModel",
          "[PointMassModel][update]") {
  CheckDataAfterUpdate<double>();
  CheckDataAfterUpdate<Vec3D>();
}

}  // namespace
}  // namespace holon
