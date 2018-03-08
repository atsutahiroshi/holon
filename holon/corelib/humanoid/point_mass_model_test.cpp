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

#include "holon/corelib/humanoid/point_mass_model.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

template <typename State>
using Model = PointMassModel<State>;

template <typename T>
void CheckInitializedMembers(const Model<T>& model, const T& expected_position,
                             const double expected_mass) {
  CHECK(model.time() == 0.0);
  CHECK(model.time_step() == Model<T>::default_time_step);
  CHECK(model.data().position == expected_position);
  CHECK(model.data().mass == expected_mass);
  CHECK(model.system().data_ptr() == model.data_ptr());
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
  auto data = createPointMassModelData(v, mass);
  Model<T> model(data);
  CheckInitializedMembers(model, v, mass);
  CHECK(model.data_ptr() == data);
  CHECK(model.system().data_ptr() == data);
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
  auto model = makePointMassModel<T>();
  REQUIRE(model.time_step() != dt);
  model.set_time_step(dt);
  CHECK(model.time_step() == Approx(dt));
}

template <typename T>
void CheckDataPtr() {
  auto data = createPointMassModelData<T>();
  auto model = makePointMassModel<T>();
  REQUIRE(model.data_ptr() != data);
  model.set_data_ptr(data);
  CHECK(model.data_ptr() == data);
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
}

template <typename T>
void CheckFactroy_1() {
  auto model = makePointMassModel<T>();
  CheckInitializedMembers(model, T(0), 1.0);
}

template <typename T>
void CheckFactroy_2() {
  auto v = Fuzzer().get<T>();
  auto model = makePointMassModel(v);
  CheckInitializedMembers(model, v, 1.0);
}

template <typename T>
void CheckFactroy_3() {
  auto v = Fuzzer().get<T>();
  auto mass = Fuzzer(0, 10).get();
  auto model = makePointMassModel(v, mass);
  CheckInitializedMembers(model, v, mass);
}

TEST_CASE("Check factory functions of PointMassModel",
          "[PointMassModel][factory]") {
  SECTION("Overloaded function 1") {
    CheckFactroy_1<double>();
    CheckFactroy_1<Vec3D>();
  }
  SECTION("Overloaded function 2") {
    CheckFactroy_2<double>();
    CheckFactroy_2<Vec3D>();
  }
  SECTION("Overloaded function 3") {
    CheckFactroy_3<double>();
    CheckFactroy_3<Vec3D>();
  }
}

}  // namespace
}  // namespace holon
