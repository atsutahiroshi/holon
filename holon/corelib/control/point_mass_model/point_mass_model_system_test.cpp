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

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

template <typename T>
using System = PointMassModelSystem<T>;

template <typename T>
System<T> prepareSystem(const T& v, double mass) {
  auto data = createPointMassModelData(v, mass);
  auto sys = makePointMassModelSystem(data);
  return sys;
}

TEST_CASE("Constructors of PointMassModelSystem",
          "[PointMassModelSystem][ctor]") {
  auto data = createPointMassModelData(kVec3DZ, 1.0);
  auto sys = makePointMassModelSystem(data);
  REQUIRE(sys.data_ptr() == data);
}

TEST_CASE("Accessors / mutators of PointMassModelSystem",
          "[PointMassModelSystem][accessor][mutator]") {
  auto data = createPointMassModelData(kVec3DZ, 1.0);
  auto sys = makePointMassModelSystem(data);

  SECTION("data reference") {
    CHECK(sys.data().mass == 1.0);
    // assignment via data() is not allowed
    // sys.data().mass = 2.0;
  }

  SECTION("data pointer") {
    auto data2 = createPointMassModelData(kVec3DZero, 2.0);
    sys.set_data_ptr(data2);
    REQUIRE(sys.data_ptr() == data2);
  }
}

TEST_CASE("operator() of PointMassModelSystem", "[PointMassModelSystem]") {}

template <typename T>
void CheckAcceleration() {
  Fuzzer fuzz(0, 10);
  auto data = createPointMassModelData<T>(fuzz.get<T>(), fuzz());
  auto sys = makePointMassModelSystem(data);
  T p, v;
  double t;
  SECTION("if not set acceleration function, returns 0") {
    CHECK(sys.acceleration(p, v, t) == T(0.0));
  }
  SECTION("if set acceleration function, returns computed acceleration") {
    auto acc = fuzz.get<T>();
    auto f_acc = [acc](const T&, const T&, const double) { return acc; };
    sys.set_acceleration(f_acc);
    CHECK(sys.acceleration(p, v, t) == acc);
  }
  SECTION("you can use data as well") {
    auto f_acc = [sys](const T&, const T&, const double) {
      return sys.data().mass * T(1.0);
    };
    sys.set_acceleration(f_acc);
    CHECK(sys.acceleration(p, v, t) == data->mass * T(1.0));
    data->mass = 3.0;
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
  auto data = createPointMassModelData<T>(fuzz.get<T>(), mass);
  auto sys = makePointMassModelSystem(data);
  T p, v;
  double t;
  SECTION("if not set force function, returns 0") {
    CHECK(sys.force(p, v, t) == T(0.0));
    CHECK(sys.acceleration(p, v, t) == T(0.0));
  }
  SECTION("if set force function, returns computed force") {
    auto force = fuzz.get<T>();
    auto f_force = [force](const T&, const T&, const double) { return force; };
    sys.set_force(f_force);
    CHECK(sys.force(p, v, t) == force);
    CHECK(sys.acceleration(p, v, t) == force / data->mass);
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
