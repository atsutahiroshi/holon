/* vec3d - 3D vector that wraps zVec3D
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

#include "holon/corelib/math/zvec3d/vec3d.hpp"

#include "catch.hpp"
#include "holon/test/util/catch/custom_matchers.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

#include <utility>

namespace holon {
namespace math {
namespace zvec3d {
namespace {

using Catch::Matchers::Equals;

TEST_CASE("zvec3d::Vec3D: constructors", "[corelib][math][Vec3D]") {
  Fuzzer fuzz;

  SECTION("default constructor should initialize with zeros") {
    Vec3D v;
    CHECK(v[0] == 0.0);
    CHECK(v[1] == 0.0);
    CHECK(v[2] == 0.0);
  }

  SECTION("constructor with three arguments") {
    double x = fuzz.get();
    double y = fuzz.get();
    double z = fuzz.get();
    Vec3D v(x, y, z);
    CHECK(v[0] == x);
    CHECK(v[1] == y);
    CHECK(v[2] == z);
  }
}

TEST_CASE("zvec3d::Vec3D: copy constructor", "[corelib][math][Vec3D]") {
  Vec3D a(1.0, 2.0, 3.0);
  Vec3D b(a);
  CHECK_THAT(b, Equals(a));
}

TEST_CASE("zvec3d::Vec3D: copy assignment operator", "[corelib][math][Vec3D]") {
  Fuzzer fuzz;
  Vec3D a, b;
  a[0] = fuzz.get();
  a[1] = fuzz.get();
  a[2] = fuzz.get();
  b = a;
  CHECK_THAT(b, Equals(a));
}

TEST_CASE("zvec3d::Vec3D: move constructor", "[corelib][math][Vec3D]") {
  Vec3D a1(2, 3, 4);
  Vec3D a2(2, 3, 4);

  Vec3D b = std::move(a1);
  CHECK_THAT(b, Equals(a1));

  auto f = [](Vec3D arg) { return arg; };
  Vec3D c = f(Vec3D(2, 3, 4));
  CHECK_THAT(c, Equals(a2));
}

TEST_CASE("zvec3d::Vec3D: move assignment operator", "[corelib][math][Vec3D]") {
  Vec3D a1(3, 4, 5);
  Vec3D a2(3, 4, 5);
  Vec3D b, c;

  b = std::move(a1);
  CHECK_THAT(b, Equals(a2));

  auto f = [](Vec3D arg) { return arg; };
  c = f(Vec3D(3, 4, 5));
  CHECK_THAT(c, Equals(a2));
}

TEST_CASE("zvec3d::Vec3D: subscript operator", "[corelib][math][Vec3D]") {
  Vec3D a;
  a[0] = 1.0;
  a[1] = 2.0;
  a[2] = 3.0;
  CHECK(a[0] == 1.0);
  CHECK(a[1] == 2.0);
  CHECK(a[2] == 3.0);
}

TEST_CASE("zvec3d::Vec3D: accessors/mutators", "[corelib][math][Vec3D]") {
  SECTION("x, y, z") {
    Vec3D a;
    Fuzzer fuzz;
    double x = fuzz.get();
    double y = fuzz.get();
    double z = fuzz.get();
    a.set_x(x);
    a.set_y(y);
    a.set_z(z);
    CHECK(a.x() == x);
    CHECK(a.y() == y);
    CHECK(a.z() == z);
  }
}

TEST_CASE("zvec3d::Vec3D: function size returns 3", "[corelib][math][Vec3D]") {
  Vec3D a;
  CHECK(a.size() == 3);
}

}  // namespace
}  // namespace zvec3d
}  // namespace math
}  // namespace holon
