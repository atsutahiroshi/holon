/* com_zmp_model - COM-ZMP model
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

#include "holon/com_zmp_model.hpp"

#include "catch.hpp"

namespace holon {
namespace {

TEST_CASE("computes acceleration based on COM-ZMP model",
          "[corelib][humanoid]") {
  ComZmpModel model;

  zVec3D pg, pz;
  zVec3DCreate(&pg, 0, 0, 0);
  zVec3DCreate(&pz, 0, 0, 0);

  zVec3D acc;
  model.ComputeAcc(&pg, &pz, &acc);
  REQUIRE(zVec3DElem(&acc, zX) == 0);
  REQUIRE(zVec3DElem(&acc, zY) == 0);
  REQUIRE(zVec3DElem(&acc, zZ) == -RK_G);
}

}  // namespace
}  // namespace holon
