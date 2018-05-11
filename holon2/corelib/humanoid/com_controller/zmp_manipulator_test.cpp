/* zmp_manipulator - ZMP manipulator class
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

#include "holon2/corelib/humanoid/com_controller/zmp_manipulator.hpp"

#include <array>
#include "holon2/corelib/common/random.hpp"
#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

using ::Catch::Matchers::ApproxEquals;
const double kTOL = 1e-10;

TEST_CASE("zmp_manipulator: check c'tor", "[ComController][ZmpManipulator]") {
  ComControllerData data;
  ZmpManipulator zmp(data);
  CHECK(zmp.data() == data);
}

TEST_CASE("zmp_manipulator: set data", "[ComController][ZmpManipulator]") {
  ComControllerData data1;
  ZmpManipulator zmp(data1);
  ComControllerData data2;
  REQUIRE(zmp.data() != data2);
  zmp.setData(data2);
  CHECK(zmp.data() == data2);
}

TEST_CASE("zmp_manipulator: accessor to control parameters",
          "[ComController][ZmpManipulator]") {
  Random<double> rnd(0, 1);
  std::array<double, 3> q1 = {rnd(), rnd(), rnd()};
  ComControllerData data;
  data.get<2>().q1 = q1;
  const ZmpManipulator zmp(data);
  for (auto i = 0; i < 3; ++i) {
    CHECK(zmp.params().q1[i] == q1[i]);
  }
}

TEST_CASE("zmp_manipulator: calculate desired ZMP",
          "[ComController][ZmpManipulator]") {
  Random<Vec3d> vec(0, 2);
  Random<Array3d> arr(0, 1);
  Random<double> rnd(0, 1);
  Vec3d p = vec(), v = vec();
  ComControllerData data;
  auto& params = data.get<2>();
  params.com_position = vec();
  params.com_velocity = vec();
  params.q1 = arr();
  params.q2 = arr();
  params.rho = rnd();
  params.dist = rnd();
  params.kr = rnd();
  data.get<0>().vhp = rnd();
  double zeta = rnd();
  ZmpManipulator zmp(data);
  auto expected_xz = zmp.calculateX(p.x(), v.x(), zeta);
  auto expected_yz = zmp.calculateY(p.y(), v.y(), zeta);
  auto expected_zz = data.get<0>().vhp;
  Vec3d expected_zmp(expected_xz, expected_yz, expected_zz);
  CHECK_THAT(zmp.calculate(p, v, zeta), ApproxEquals(expected_zmp, kTOL));
}

TEST_CASE("zmp_manipulator: ", "[ComController][ZmpManipulator]") {}

}  // namespace
}  // namespace holon
