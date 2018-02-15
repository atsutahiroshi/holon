/* custom_matchers - Custom matchers for Catch
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

#ifndef HOLON_TEST_UTIL_CATCH_CUSTOM_MATCHERS_HPP_
#define HOLON_TEST_UTIL_CATCH_CUSTOM_MATCHERS_HPP_

#include <zeo/zeo_vec3d.h>
#include "holon/corelib/math/zvec3dwrap/vec3d.hpp"

#include <sstream>
#include <string>
#include "third_party/catch/catch.hpp"

namespace Catch {
namespace Detail {

inline std::string stringify(const zVec3D& v) {
  std::ostringstream ss;
  ss << "( ";
  ss << zVec3DElem(&v, zX) << ", " << zVec3DElem(&v, zY) << ", "
     << zVec3DElem(&v, zZ);
  ss << " )";
  return ss.str();
}

inline std::string stringfy(const holon::zvec3d::Vec3D& v) {
  std::ostringstream ss;
  ss << "( ";
  ss << v.x() << ", " << v.y() << ", " << v.z();
  ss << " )";
  return ss.str();
}

}  // namespace Detail

template <>
struct StringMaker<zVec3D> {
  static std::string convert(const zVec3D& v) {
    return ::Catch::Detail::stringify(v);
  }
};

template <>
struct StringMaker<holon::zvec3d::Vec3D> {
  static std::string convert(const holon::zvec3d::Vec3D& v) {
    return ::Catch::Detail::stringify(v.get());
    // return ::Catch::Detail::stringify(v);
  }
};

namespace Matchers {
namespace zvec3d {

class EqualsMatcher : public MatcherBase<zVec3D> {
 public:
  explicit EqualsMatcher(const zVec3D& comparator) : m_comparator(comparator) {}

  bool match(const zVec3D& v) const override {
    if (zVec3DEqual(const_cast<zVec3D*>(&m_comparator),
                    const_cast<zVec3D*>(&v)))
      return true;
    else
      return false;
  }

  std::string describe() const override {
    std::ostringstream ss;
    return "Equals: " + ::Catch::Detail::stringify(m_comparator);
  }

 private:
  const zVec3D& m_comparator;
};

}  // namespace zvec3d

namespace vec3d {
namespace zvec3d {

using holon::zvec3d::Vec3D;

class EqualsMatcher : public MatcherBase<Vec3D> {
 public:
  explicit EqualsMatcher(const Vec3D& comparator)
      : m_comparator(comparator),
        m_epsilon(std::numeric_limits<float>::epsilon() * 100) {}
  bool match(const Vec3D& v) const override {
    for (std::size_t i = 0; i < v.size(); ++i) {
      if (std::fabs(m_comparator[i] - v[i]) > m_epsilon) {
        return false;
      }
    }
    return true;
  }
  std::string describe() const override {
    return "Equals: " + ::Catch::Detail::stringify(m_comparator);
  }

 private:
  const Vec3D& m_comparator;
  double m_epsilon;
};

}  // namespace zvec3d
}  // namespace vec3d

inline zvec3d::EqualsMatcher Equals(const zVec3D& comparator) {
  return zvec3d::EqualsMatcher(comparator);
}

inline vec3d::zvec3d::EqualsMatcher Equals(
    const holon::zvec3d::Vec3D& comparator) {
  return vec3d::zvec3d::EqualsMatcher(comparator);
}

}  // namespace Matchers
}  // namespace Catch

#endif  // HOLON_TEST_UTIL_CATCH_CUSTOM_MATCHERS_HPP_
