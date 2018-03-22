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

#include "holon/corelib/control/point_mass_model/point_mass_model_data.hpp"

#include <type_traits>
#include "holon/corelib/math/vec3d.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

template <typename T>
using Data = experimental::PointMassModelData<T>;

template <typename T>
void CheckMembers(const Data<T>& data, const T& expected_p0,
                  const double expected_mass) {
  T zero(0.0);
  CHECK(data().mass == expected_mass);
  CHECK(data().position == expected_p0);
  CHECK(data().velocity == zero);
  CHECK(data().acceleration == zero);
  CHECK(data().force == zero);
}

template <typename T>
void CheckCtor_0() {
  Data<T> data;
  CheckMembers(data, T(0), Data<T>::default_mass);
}

template <typename T>
void CheckCtor_1() {
  auto v = Fuzzer().get<T>();
  Data<T> data(v);
  CheckMembers(data, v, Data<T>::default_mass);
}

template <typename T>
void CheckCtor_2() {
  double mass = Fuzzer(0, 10).get();
  auto v = Fuzzer().get<T>();
  Data<T> data(v, mass);
  CheckMembers(data, v, mass);
}

TEST_CASE("exp:Constructor of PointMassModelData",
          "[PointMassModelData][ctor]") {
  SECTION("Default constructor") {
    CheckCtor_0<double>();
    CheckCtor_0<Vec3D>();
  }
  SECTION("Overloaded constructor 1") {
    CheckCtor_1<double>();
    CheckCtor_1<Vec3D>();
  }
  SECTION("Overloaded constructor 2") {
    CheckCtor_2<double>();
    CheckCtor_2<Vec3D>();
  }
}

}  // namespace
}  // namespace holon
