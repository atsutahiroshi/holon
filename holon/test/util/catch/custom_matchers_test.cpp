/* custom_matchers_test - Custom matchers for Catch
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

#include "holon/test/util/catch/custom_matchers.hpp"
#include "catch.hpp"

namespace holon {
namespace {

using Catch::Matchers::Equals;

TEST_CASE("matchers for zVec3D", "[test][util][catch]") {
  zVec3D v = {0.1, 0.2, 0.3};
  zVec3D v1 = {0.1, 0.2, 0.3};
  zVec3D v2 = {0.1, 0.1, 0.1};

  CHECK_THAT(v, Equals(v1));
  CHECK_THAT(v, !Equals(v2));
}

}  // namespace

namespace {

TEST_CASE("matchers for zvec3d::Vec3D", "[test][util][catch]") {
  zvec3d::Vec3D v(0.1, 0.2, 0.3);
  zvec3d::Vec3D v1(0.1, 0.2, 0.3);
  zvec3d::Vec3D v2(0.1, 0.1, 0.1);

  CHECK_THAT(v, Equals(v1));
  CHECK_THAT(v, !Equals(v2));

  v1 = {0.1, 0.2, (0.1 + 0.2)};
  CHECK_THAT(v, Equals(v1));
}

}  // namespace

}  // namespace holon
