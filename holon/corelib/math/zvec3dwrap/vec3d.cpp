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

#include "holon/corelib/math/zvec3dwrap/vec3d.hpp"

#include <iomanip>
#include <sstream>
#include <string>

namespace holon {
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

Vec3D Vec3D::opposite() const { return Vec3D(-m_v.e[0], -m_v.e[1], -m_v.e[2]); }

bool Vec3D::match(const Vec3D& other) const {
  return zVec3DMatch(const_cast<zVec3D*>(get_ptr()),
                     const_cast<zVec3D*>(other.get_ptr()));
}

bool Vec3D::equal(const Vec3D& other) const {
  return zVec3DEqual(const_cast<zVec3D*>(get_ptr()),
                     const_cast<zVec3D*>(other.get_ptr()));
}

bool Vec3D::istiny(double tol) const {
  return zVec3DIsTol(const_cast<zVec3D*>(get_ptr()), tol);
}

bool Vec3D::isnan() const {
  return zVec3DIsNan(const_cast<zVec3D*>(get_ptr()));
}

std::string Vec3D::str() const {
  std::stringstream ss;
  ss << "( ";
  ss << x() << ", " << y() << ", " << z();
  ss << " )";
  return ss.str();
}

std::string Vec3D::data(const std::string& delim, int precision) const {
  std::stringstream ss;
  ss << std::scientific << std::setprecision(precision);
  ss << x() << delim << y() << delim << z();
  return ss.str();
}

Vec3D Vec3D::add(const Vec3D& rhs) const {
  return Vec3D(m_v.e[0] + rhs[0], m_v.e[1] + rhs[1], m_v.e[2] + rhs[2]);
}

Vec3D Vec3D::add(double rhs) const {
  return Vec3D(m_v.e[0] + rhs, m_v.e[1] + rhs, m_v.e[2] + rhs);
}

Vec3D Vec3D::sub(const Vec3D& rhs) const {
  return Vec3D(m_v.e[0] - rhs[0], m_v.e[1] - rhs[1], m_v.e[2] - rhs[2]);
}

Vec3D Vec3D::sub(double rhs) const {
  return Vec3D(m_v.e[0] - rhs, m_v.e[1] - rhs, m_v.e[2] - rhs);
}

Vec3D Vec3D::mul(double rhs) const {
  return Vec3D(m_v.e[0] * rhs, m_v.e[1] * rhs, m_v.e[2] * rhs);
}

Vec3D Vec3D::div(double rhs) const {
  if (rhs == 0) {
    ZRUNWARN("cannot divide by zero value");
    return *this;
  }
  rhs = 1.0 / rhs;
  return mul(rhs);
}

double Vec3D::dot(const Vec3D& rhs) const {
  double retval = 0;
  for (std::size_t i = 0; i < size(); ++i) retval += (*this)[i] * rhs[i];
  return retval;
}

Vec3D Vec3D::cross(const Vec3D& rhs) const {
  double x_, y_, z_;
  x_ = this->y() * rhs.z() - this->z() * rhs.y();
  y_ = this->z() * rhs.x() - this->x() * rhs.z();
  z_ = this->x() * rhs.y() - this->y() * rhs.x();
  return Vec3D(x_, y_, z_);
}

Vec3D::iterator Vec3D::begin() {
  return Vec3DIterator<double>(const_cast<double*>(m_v.e));
}

Vec3D::iterator Vec3D::end() {
  return Vec3DIterator<double>(const_cast<double*>(m_v.e + size()));
}

Vec3D::const_iterator Vec3D::begin() const {
  return Vec3DIterator<const double>(const_cast<double*>(m_v.e));
}

Vec3D::const_iterator Vec3D::end() const {
  return Vec3DIterator<const double>(const_cast<double*>(m_v.e + size()));
}

Vec3D::const_iterator Vec3D::cbegin() const {
  return Vec3DIterator<const double>(const_cast<double*>(m_v.e));
}

Vec3D::const_iterator Vec3D::cend() const {
  return Vec3DIterator<const double>(const_cast<double*>(m_v.e + size()));
}

std::ostream& operator<<(std::ostream& os, const Vec3D& v) {
  os << "( ";
  os << v[0] << ", " << v[1] << ", " << v[2];
  os << " )";
  return os;
}

}  // namespace zvec3d
}  // namespace holon
