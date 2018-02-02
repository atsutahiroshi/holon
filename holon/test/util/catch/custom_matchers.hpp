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

#include <sstream>
#include <string>
#include "third_party/catch/catch.hpp"

namespace Catch {
namespace Detail {

inline std::string zVec3DToString(const zVec3D* v) {
  std::ostringstream ss;
  ss << "( ";
  if (!v) {
    ss << "null vector";
  } else {
    ss << zVec3DElem(v, zX) << ", " << zVec3DElem(v, zY) << ", "
       << zVec3DElem(v, zZ);
  }
  ss << " )";
  return ss.str();
}

}  // namespace Detail

template <>
struct StringMaker<zVec3D*> {
  static std::string convert(zVec3D* v) {
    return ::Catch::Detail::zVec3DToString(v);
  }
};

class zVec3DMatcher : public MatcherBase<zVec3D*> {
 public:
  explicit zVec3DMatcher(zVec3D* comparator) : comparator_(comparator) {}

  bool match(zVec3D* v) const override {
    if (zVec3DEqual(comparator_, v))
      return true;
    else
      return false;
  }

  std::string describe() const override {
    std::ostringstream ss;
    return "Equals: " + ::Catch::Detail::zVec3DToString(comparator_);
  }

 private:
  zVec3D* comparator_;
};

inline zVec3DMatcher Equals(zVec3D* comparator) {
  return zVec3DMatcher(comparator);
}

}  // namespace Catch

#endif  // HOLON_TEST_UTIL_CATCH_CUSTOM_MATCHERS_HPP_
