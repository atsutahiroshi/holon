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

namespace holon {
namespace math {
namespace zvec3d {

Vec3D::Vec3D() { zVec3DClear(&m_v); }
Vec3D::Vec3D(double t_x, double t_y, double t_z) {
  zVec3DCreate(&m_v, t_x, t_y, t_z);
}

Vec3D& Vec3D::set_x(double t_x) {
  zVec3DSetElem(&m_v, zX, t_x);
  return *this;
}

Vec3D& Vec3D::set_y(double t_y) {
  zVec3DSetElem(&m_v, zY, t_y);
  return *this;
}

Vec3D& Vec3D::set_z(double t_z) {
  zVec3DSetElem(&m_v, zZ, t_z);
  return *this;
}

}  // namespace zvec3d
}  // namespace math
}  // namespace holon
