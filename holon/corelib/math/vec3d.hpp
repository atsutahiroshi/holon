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

#ifndef HOLON_MATH_VEC3D_HPP_
#define HOLON_MATH_VEC3D_HPP_

#include "holon/corelib/math/zvec3d/vec3d.hpp"

namespace holon {

using zvec3d::Vec3D;

extern const Vec3D kVec3DZero;
extern const Vec3D kVec3DX;
extern const Vec3D kVec3DY;
extern const Vec3D kVec3DZ;

}  // namespace holon

#endif  // HOLON_MATH_VEC3D_HPP_
