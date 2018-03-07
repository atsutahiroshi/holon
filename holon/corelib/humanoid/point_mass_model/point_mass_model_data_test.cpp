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
void CheckConstructor_1() {
  using Data = PointMassModelData<T>;
  Data data;
  T zero(0.0);

  CHECK(data.mass == Data::default_mass);
  CHECK(data.position == zero);
  CHECK(data.velocity == zero);
  CHECK(data.acceleration == zero);
  CHECK(data.force == zero);
}

template <typename T>
void CheckConstructor_2() {
  using Data = PointMassModelData<T>;
  double mass = Fuzzer(0, 10).get();
  Data data(mass);
  T zero(0.0);

  CHECK(data.mass == mass);
  CHECK(data.position == zero);
  CHECK(data.velocity == zero);
  CHECK(data.acceleration == zero);
  CHECK(data.force == zero);
}

template <typename T>
void CheckConstructor_3() {
  using Data = PointMassModelData<T>;
  double mass = Fuzzer(0, 10).get();
  T v(Fuzzer().get<T>());
  Data data(mass, v);
  T zero(0.0);

  CHECK(data.mass == mass);
  CHECK(data.position == v);
  CHECK(data.velocity == zero);
  CHECK(data.acceleration == zero);
  CHECK(data.force == zero);
}

TEST_CASE("Constructor of PointMassModelData", "[PointMassModelData][ctor]") {
  SECTION("Default constructor") {
    CheckConstructor_1<double>();
    CheckConstructor_1<Vec3D>();
  }
  SECTION("With one argument") {
    CheckConstructor_2<double>();
    CheckConstructor_2<Vec3D>();
  }
  SECTION("With two arguments") {
    CheckConstructor_3<double>();
    CheckConstructor_3<Vec3D>();
  }
}

}  // namespace
}  // namespace holon
