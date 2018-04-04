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

#include <memory>
#include <type_traits>
#include "holon/corelib/math/vec3d.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

template <typename T>
using RawData = PointMassModelRawData<T>;
template <typename T>
using Data = PointMassModelData<T>;

template <typename T>
void CheckMembers(const Data<T>& data, const T& expected_p0,
                  const double expected_mass) {
  T zero(0.0);
  CHECK(data.get().mass == expected_mass);
  CHECK(data.get().position == expected_p0);
  CHECK(data.get().velocity == zero);
  CHECK(data.get().acceleration == zero);
  CHECK(data.get().force == zero);
}

template <typename T>
void CheckCtor_0() {
  Data<T> data;
  CheckMembers(data, T(0), RawData<T>::default_mass);
}

template <typename T>
void CheckCtor_1() {
  double mass = Fuzzer(0, 10).get();
  auto v = Fuzzer().get<T>();
  RawData<T> rawdata{mass, v, T{0}, T{0}, T{0}};
  Data<T> data(rawdata);
  CheckMembers(data, v, mass);
  REQUIRE(data.template get_ptr<0>().get() != &rawdata);
}

template <typename T>
void CheckCtor_2() {
  double mass = Fuzzer(0, 10).get();
  auto v = Fuzzer().get<T>();
  auto rawdata_p = std::make_shared<RawData<T>>();
  rawdata_p->mass = mass;
  rawdata_p->position = v;
  rawdata_p->velocity = T{0};
  rawdata_p->acceleration = T{0};
  rawdata_p->force = T{0};
  Data<T> data(rawdata_p);
  CheckMembers(data, v, mass);
  REQUIRE(data.template get_ptr<0>() == rawdata_p);
}

template <typename T>
void CheckCtor_3() {
  auto v = Fuzzer().get<T>();
  Data<T> data(v);
  CheckMembers(data, v, RawData<T>::default_mass);
}

template <typename T>
void CheckCtor_4() {
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
  SECTION("Overloaded constructor 3") {
    CheckCtor_3<double>();
    CheckCtor_3<Vec3D>();
  }
  SECTION("Overloaded constructor 4") {
    CheckCtor_4<double>();
    CheckCtor_4<Vec3D>();
  }
}

}  // namespace
}  // namespace holon
