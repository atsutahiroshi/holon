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

#ifndef HOLON_MATH_ZVEC3D_VEC3D_HPP_
#define HOLON_MATH_ZVEC3D_VEC3D_HPP_

#include <zeo/zeo_vec3d.h>
#include <cstddef>

namespace holon {
namespace zVec3DWrap {

class Vec3D {
 public:
  // constructors
  Vec3D();
  Vec3D(double t_x, double t_y, double t_z);

  // special member functions
  virtual ~Vec3D() noexcept = default;
  Vec3D(const Vec3D&) = default;
  Vec3D(Vec3D&&) noexcept = default;
  Vec3D& operator=(const Vec3D&) = default;
  Vec3D& operator=(Vec3D&&) noexcept = default;

  // array subscript operators
  inline double& operator[](std::size_t idx) { return m_v.e[idx]; }
  inline const double& operator[](std::size_t idx) const { return m_v.e[idx]; }

  // accessors
  inline zVec3D* get_ptr() noexcept { return &m_v; }
  inline const zVec3D get() const noexcept { return m_v; }
  inline double x() const noexcept { return zVec3DElem(&m_v, zX); }
  inline double y() const noexcept { return zVec3DElem(&m_v, zY); }
  inline double z() const noexcept { return zVec3DElem(&m_v, zZ); }

  // mutators
  Vec3D& set_x(double t_x);
  Vec3D& set_y(double t_y);
  Vec3D& set_z(double t_z);

  // member functions
  inline std::size_t size() const noexcept { return 3; }
  inline Vec3D clone() const { return Vec3D(*this); }
  Vec3D opposite() const;
  inline void clear() { zVec3DClear(&m_v); }

  // arithmetic member functions
  Vec3D add(const Vec3D& rhs) const;
  Vec3D add(double rhs) const;
  Vec3D sub(const Vec3D& rhs) const;
  Vec3D sub(double rhs) const;
  Vec3D mul(double rhs) const;
  Vec3D div(double rhs) const;

  // arithmetic unary operators
  inline Vec3D operator+() const { return clone(); }
  inline Vec3D operator-() const { return opposite(); }

  // arithmetic binary operators
  inline Vec3D operator+(const Vec3D& rhs) const { return add(rhs); }
  inline Vec3D operator+(double rhs) const { return add(rhs); }
  inline Vec3D operator-(const Vec3D& rhs) const { return sub(rhs); }
  inline Vec3D operator-(double rhs) const { return sub(rhs); }
  inline Vec3D operator*(double rhs) const { return mul(rhs); }
  inline Vec3D operator/(double rhs) const { return div(rhs); }

 private:
  zVec3D m_v;
};

// non-member arithmetic operators
inline Vec3D operator+(double lhs, const Vec3D& rhs) { return rhs.add(lhs); }
inline Vec3D operator-(double lhs, const Vec3D& rhs) { return -rhs.sub(lhs); }
inline Vec3D operator*(double lhs, const Vec3D& rhs) { return rhs.mul(lhs); }

}  // namespace zvec3d
}  // namespace holon

#endif  // HOLON_MATH_ZVEC3D_VEC3D_HPP_
