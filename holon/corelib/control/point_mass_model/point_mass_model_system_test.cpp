/* point_mass_model_system - System definition of point mass model
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

#include "holon/corelib/control/point_mass_model/point_mass_model_system.hpp"

#include <utility>

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

template <typename T>
using System = PointMassModelSystem<T>;

template <typename T>
System<T> prepareSystem(const T& v, double mass) {
  auto data = make_data<PointMassModelData<T>>(v, mass);
  auto sys = make_system<PointMassModelSystem, T>(data);
  return sys;
}

template <typename T>
struct get_element_type {
  using type = typename std::remove_reference<T>::type::element_type;
};
template <typename T>
using get_element_type_t = typename get_element_type<T>::type;

TEST_CASE("Constructors of PointMassModelSystem",
          "[PointMassModelSystem][ctor]") {
  auto data = make_data<PointMassModelData>(kVec3DZ, 1.0);
  auto sys = make_system<PointMassModelSystem>(data);
  REQUIRE(std::is_same<get_element_type_t<decltype(data.get_ptr<0>())>,
                       PointMassModelRawData<Vec3D>>::value);
  REQUIRE(std::is_same<get_element_type_t<decltype(sys.data().get_ptr<0>())>,
                       PointMassModelRawData<Vec3D>>::value);
  REQUIRE(sys.data() == data);
}

TEST_CASE("Accessors / mutators of PointMassModelSystem",
          "[PointMassModelSystem][accessor][mutator]") {
  auto data = make_data<PointMassModelData>(kVec3DZ, 1.0);
  auto sys = make_system<PointMassModelSystem>(data);
  SECTION("data reference") {
    CHECK(sys.data().get().mass == 1.0);
    // assignmen via data() is not allowed
    // sys.data().get().mass = 2.0;
  }
  SECTION("data pointer") {
    auto data2 = make_data<PointMassModelData>(kVec3DZero, 2.0);
    sys.set_data(data2);
    REQUIRE(sys.data() == data2);
  }
}

template <typename T>
void CheckAcceleration() {
  Fuzzer fuzz(0, 10);
  auto data = make_data<PointMassModelData>(fuzz.get<T>(), fuzz());
  auto sys = make_system<PointMassModelSystem, T>(data);
  T p, v;
  double t;
  SECTION("if acceleration function is not set, returns 0") {
    CHECK(sys.acceleration(p, v, t) == T(0.0));
  }
  SECTION("if acceleration function is set, returns computed acceleration") {
    auto acc = fuzz.get<T>();
    auto f_acc = [acc](const T&, const T&, const double) { return acc; };
    sys.set_acceleration(f_acc);
    CHECK(sys.acceleration(p, v, t) == acc);
  }
  SECTION("you can use data as well") {
    auto f_acc = [sys](const T&, const T&, const double) {
      return sys.data().get().mass * T(1.0);
    };
    sys.set_acceleration(f_acc);
    CHECK(sys.acceleration(p, v, t) == data.get().mass * T(1.0));
    data.get().mass = 3.0;
    CHECK(sys.acceleration(p, v, t) == 3.0 * T(1.0));
  }
}

TEST_CASE("PointMassModelSystem::acceleration", "[PointMassModelSystem]") {
  SECTION("State type is double") { CheckAcceleration<double>(); }
  SECTION("State type is Vec3D") { CheckAcceleration<Vec3D>(); }
}

template <typename T>
void CheckForce() {
  Fuzzer fuzz(0, 10);
  auto mass = 2.0;
  auto data = make_data<PointMassModelData<T>>(fuzz.get<T>(), mass);
  auto sys = make_system<PointMassModelSystem, T>(data);
  T p, v;
  double t;
  SECTION("if force function is not set, returns 0") {
    CHECK(sys.force(p, v, t) == T{0.0});
    CHECK(sys.acceleration(p, v, t) == T{0.0});
  }
  SECTION("if force function is set, returns computed force") {
    auto force = fuzz.get<T>();
    auto f_force = [force](const T&, const T&, const double) { return force; };
    sys.set_force(f_force);
    CHECK(sys.force(p, v, t) == force);
    CHECK(sys.acceleration(p, v, t) == force / data.get().mass);
  }
}

TEST_CASE("PointMassModelSystem::force", "[PointMassModelSystem]") {
  SECTION("Case where state type is double") { CheckForce<double>(); }
  SECTION("Case where state type is Vec3D") { CheckForce<Vec3D>(); }
}

template <typename T>
void CheckSetConstForce() {
  auto sys = prepareSystem(T(0.0), 1.0);
  auto force = Fuzzer().get<T>();
  sys.setConstForce(force);
  T p, v;
  double t;
  CHECK(sys.force(p, v, t) == force);
}

TEST_CASE("PointMassModelSytem::setConstForce", "[PointMassModelSystem]") {
  SECTION("Case where state type is double") { CheckSetConstForce<double>(); }
  SECTION("Case where state type is Vec3D") { CheckSetConstForce<Vec3D>(); }
}

}  // namespace
}  // namespace holon
