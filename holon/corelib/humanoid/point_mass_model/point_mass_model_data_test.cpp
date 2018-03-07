/* point_mass_model_data - Data for point mass model
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

#include "holon/corelib/humanoid/point_mass_model/point_mass_model_data.hpp"

#include "holon/corelib/math/vec3d.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

template <typename T>
using Data = PointMassModelData<T>;

template <typename T>
void CheckMembers(const PointMassModelData<T>& data, const double expected_mass,
                  const T& expected_p0) {
  T zero(0.0);
  CHECK(data.mass == expected_mass);
  CHECK(data.position == expected_p0);
  CHECK(data.velocity == zero);
  CHECK(data.acceleration == zero);
  CHECK(data.force == zero);
}

template <typename T>
void CheckConstructor_1() {
  Data<T> data;
  CheckMembers(data, Data<T>::default_mass, T(0));
}

template <typename T>
void CheckConstructor_2() {
  double mass = Fuzzer(0, 10).get();
  Data<T> data(mass);
  CheckMembers(data, mass, T(0));
}

template <typename T>
void CheckConstructor_3() {
  double mass = Fuzzer(0, 10).get();
  T v(Fuzzer().get<T>());
  Data<T> data(mass, v);
  CheckMembers(data, mass, v);
}

TEST_CASE("Constructor of PointMassModelData", "[PointMassModelData][ctor]") {
  SECTION("Default constructor") {
    CheckConstructor_1<double>();
    CheckConstructor_1<Vec3D>();
  }
  SECTION("Overloaded constructor 1") {
    CheckConstructor_2<double>();
    CheckConstructor_2<Vec3D>();
  }
  SECTION("Overloaded constructor 2") {
    CheckConstructor_3<double>();
    CheckConstructor_3<Vec3D>();
  }
}

template <typename T>
void CheckFactory_1() {
  auto data = createPointMassModelData<T>();
  CheckMembers(*data, Data<T>::default_mass, T(0));
}

template <typename T>
void CheckFactory_2() {
  double mass = Fuzzer(0, 10).get();
  auto data = createPointMassModelData<T>(mass);
  CheckMembers(*data, mass, T(0));
}

template <typename T>
void CheckFactory_3() {
  double mass = Fuzzer(0, 10).get();
  auto v = Fuzzer().get<T>();
  auto data = createPointMassModelData<T>(mass, v);
  CheckMembers(*data, mass, v);
}

TEST_CASE("Check factory functions of PointMassModelData",
          "[PointMassModelData]") {
  SECTION("Overloaded function 1") {
    CheckFactory_1<double>();
    CheckFactory_1<Vec3D>();
  }
  SECTION("Overloaded function 2") {
    CheckFactory_2<double>();
    CheckFactory_2<Vec3D>();
  }
  SECTION("Overloaded function 3") {
    CheckFactory_3<double>();
    CheckFactory_3<Vec3D>();
  }
}

}  // namespace
}  // namespace holon
