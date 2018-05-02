/* optional - type aliases of optional-lite
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of holon.
 *
 * Holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "holon2/corelib/common/optional.hpp"

#include <cstdlib>
#include "holon2/corelib/math/vec.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

optional<int> to_int(char const* const text) {
  char* pos = NULL;
  const int value = strtol(text, &pos, 0);

  return pos == text ? nullopt : optional<int>(value);
}

TEST_CASE("optional: Example usage", "[optional]") {
  auto a = to_int("42");
  auto b = to_int("Nan");
  CHECK(a == 42);
  CHECK(b == nullopt);
}

TEST_CASE("optional: use with Vec3d", "[optional]") {
  Vec3d z(0, 0, 0);
  optional<Vec3d> ov;
  CHECK(ov == nullopt);
  Vec3d v;
  v << 1, 2, 3;
  ov = v;
  CHECK(ov == v);
  CHECK(ov.value() == v);
  CHECK(ov.value()[0] == 1);
  CHECK(ov.value()[1] == 2);
  CHECK(ov.value()[2] == 3);
}

}  // namespace
}  // namespace holon
