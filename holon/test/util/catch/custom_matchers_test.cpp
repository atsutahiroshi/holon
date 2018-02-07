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

#include <vector>

#include "catch.hpp"

namespace holon {
namespace {

TEST_CASE("matcher for zVec3D class", "[test][util][catch]") {
  zVec3D v1 = {0.1, 0.2, 0.3};
  zVec3D v2 = {0.1, 0.1, 0.1};

  CHECK_THAT(&v1, Catch::Matchers::Equals(&v1));
  CHECK_THAT(&v1, Catch::Matchers::NotEqual(&v2));
}

}  // namespace
}  // namespace holon
