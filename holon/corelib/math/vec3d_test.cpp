/* vec3d - 3D vector
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

#include "holon/corelib/math/vec3d.hpp"
#include "holon/test/util/catch/custom_matchers.hpp"

namespace holon {
namespace {

using Catch::Matchers::Equals;

TEST_CASE("Vec3D: check constants") {
  CHECK_THAT(kVec3DZero, Equals(Vec3D(0, 0, 0)));
  CHECK_THAT(kVec3DX, Equals(Vec3D(1, 0, 0)));
  CHECK_THAT(kVec3DY, Equals(Vec3D(0, 1, 0)));
  CHECK_THAT(kVec3DZ, Equals(Vec3D(0, 0, 1)));
}

}  // namespace
}  // namespace holon
