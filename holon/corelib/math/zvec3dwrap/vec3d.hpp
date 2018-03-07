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
#include <iostream>
#include <iterator>
#include <string>

namespace holon {
namespace zvec3d {

template <typename T>
class Vec3DIterator;

class Vec3D {
 public:
  // constructors
  Vec3D();
  explicit Vec3D(double t_v);
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
  inline const zVec3D* get_ptr() const noexcept { return &m_v; }
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

  // functions to investigate equality
  bool match(const Vec3D& rhs) const;
  bool equal(const Vec3D& rhs) const;

  // check if it is tiny
  bool istiny(double tol = zTOL) const;

// TODO(*): remove this when <math.h> is completely eliminated
#if defined(isnan)
#undef isnan
#endif
  // check if it includes NaN or Inf component
  bool isnan() const;

  // functions to make string
  std::string str() const;
  std::string data(const std::string& delim = " ", int precision = 10) const;
  inline std::string data(int precision) const { return data(" ", precision); }

  // arithmetic member functions
  Vec3D add(const Vec3D& rhs) const;
  Vec3D add(double rhs) const;
  Vec3D sub(const Vec3D& rhs) const;
  Vec3D sub(double rhs) const;
  Vec3D mul(double rhs) const;
  Vec3D div(double rhs) const;

  // inner / outer product
  double dot(const Vec3D& rhs) const;
  Vec3D cross(const Vec3D& rhs) const;

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

  // relational operators
  inline bool operator==(const Vec3D& rhs) const { return equal(rhs); }
  inline bool operator!=(const Vec3D& rhs) const { return !(*this == rhs); }

  // iterators
  using iterator = Vec3DIterator<double>;
  using const_iterator = Vec3DIterator<const double>;
  iterator begin();
  iterator end();
  const_iterator begin() const;
  const_iterator end() const;
  const_iterator cbegin() const;
  const_iterator cend() const;

 private:
  zVec3D m_v;
};

// iterator for Vec3D
template <typename T>
class Vec3DIterator {
 public:
  // iterator traits
  using iterator_category = std::forward_iterator_tag;
  using value_type = T;
  using difference_type = std::ptrdiff_t;
  using pointer = T*;
  using reference = T&;

  // default constructible
  Vec3DIterator() = default;
  explicit Vec3DIterator(pointer ptr) : m_ptr(ptr) {}

  // dereferenceable
  reference operator*() const { return *m_ptr; }

  // pre- and post-incrementable
  Vec3DIterator& operator++() {
    ++m_ptr;
    return *this;
  }
  Vec3DIterator operator++(int) {
    Vec3DIterator tmp = *this;
    ++m_ptr;
    return tmp;
  }

  // equality / inequality
  bool operator==(const Vec3DIterator& rhs) { return m_ptr == rhs.m_ptr; }
  bool operator!=(const Vec3DIterator& rhs) { return !(*this == rhs); }

 private:
  pointer m_ptr{nullptr};
};

// non-member arithmetic operators
inline Vec3D operator+(double lhs, const Vec3D& rhs) { return rhs.add(lhs); }
inline Vec3D operator-(double lhs, const Vec3D& rhs) { return -rhs.sub(lhs); }
inline Vec3D operator*(double lhs, const Vec3D& rhs) { return rhs.mul(lhs); }

// non-member arithmetic functions
inline Vec3D cat(const Vec3D& v1, double k, const Vec3D& v2) {
  return v1 + k * v2;
}

// stream insertion
std::ostream& operator<<(std::ostream& os, const Vec3D& v);

}  // namespace zvec3d
}  // namespace holon

#endif  // HOLON_MATH_ZVEC3D_VEC3D_HPP_
