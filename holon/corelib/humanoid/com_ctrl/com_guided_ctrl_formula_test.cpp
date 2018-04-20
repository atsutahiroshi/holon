/* com_guided_ctrl_formula - Formulae for COM-guided control
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

#include "holon/corelib/humanoid/com_ctrl/com_guided_ctrl_formula.hpp"

#include <cure/cure_misc.h>
#include "holon/corelib/humanoid/com_ctrl.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace com_guided_ctrl_formula {
namespace {

using index_symbols::_X;
using index_symbols::_Y;
using index_symbols::_Z;

TEST_CASE("Check computation of desired ZMP position",
          "[com_guided_ctrl_formula][desired_zmp_position]") {
  Fuzzer fuzz(0, 2);
  auto p = fuzz.get<Vec3D>();
  auto v = fuzz.get<Vec3D>();
  auto pd = fuzz.get<Vec3D>();
  auto vd = fuzz.get<Vec3D>();
  auto q1 = Array3d{fuzz(), fuzz(), fuzz()};
  auto q2 = Array3d{fuzz(), fuzz(), fuzz()};
  auto rho = fuzz();
  auto dist = fuzz();
  auto kr = fuzz();
  auto vhp = fuzz();
  auto zeta = fuzz();
  auto expected_xz = desired_zmp_position_x(p[_X], v[_X], pd[_X], vd[_X],
                                            q1[_X], q2[_X], zeta);
  auto expected_yz = desired_zmp_position_y(p[_Y], v[_Y], pd[_Y], q1[_Y],
                                            q2[_Y], rho, dist, kr, zeta);
  auto expected_zz = vhp;
  auto expected_zmp = Vec3D{expected_xz, expected_yz, expected_zz};
  SECTION("Overloaded function 1") {
    CHECK(desired_zmp_position(p, v, pd, vd, q1, q2, rho, dist, kr, vhp,
                               zeta) == expected_zmp);
  }
  SECTION("Overloaded function 2") {
    ComCtrlParamsRawData data;
    data.com_position = pd;
    data.com_velocity = vd;
    data.q1 = q1;
    data.q2 = q2;
    data.rho = rho;
    data.dist = dist;
    data.kr = kr;
    data.vhp = vhp;
    CHECK(desired_zmp_position(p, v, data, zeta) == expected_zmp);
  }
}

}  // namespace
}  // namespace com_guided_ctrl_formula
}  // namespace holon
