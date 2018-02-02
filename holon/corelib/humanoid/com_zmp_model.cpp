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

#include "holon/corelib/humanoid/com_zmp_model.hpp"

#include <roki/rk_g.h>

namespace holon {

ComZmpModel::ComZmpModel() {}

ComZmpModel::~ComZmpModel() {}

double ComZmpModel::ComputeZetaSqr(const zVec3D* com_position) const {
  if (zIsTiny(zVec3DElem(com_position, zZ)) ||
      zVec3DElem(com_position, zZ) < 0) {
    ZRUNERROR("The COM height must be positive. (given: %f)",
              zVec3DElem(com_position, zZ));
    return 0.0;
  }
  return RK_G / zVec3DElem(com_position, zZ);
}

double ComZmpModel::ComputeZeta(const zVec3D* com_position) const {
  return sqrt(ComputeZetaSqr(com_position));
}

zVec3D* ComZmpModel::ComputeAcceleration(const zVec3D* com_position,
                                         const zVec3D* zmp_position,
                                         zVec3D* com_acceleration) const {
  zVec3D g = {{0, 0, RK_G}};
  // TODO(*): remove const_cast when own math library is implemented
  zVec3DSub(const_cast<zVec3D*>(com_position),
            const_cast<zVec3D*>(zmp_position), com_acceleration);
  zVec3DMulDRC(com_acceleration, ComputeZetaSqr(com_position));
  zVec3DSubDRC(com_acceleration, &g);
  return com_acceleration;
}

}  // namespace holon
