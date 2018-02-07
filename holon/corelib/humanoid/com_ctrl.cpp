/* com_ctrl - COM Controller
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

#include "holon/corelib/humanoid/com_ctrl.hpp"

namespace holon {

ComCtrl::ComCtrl() : m_x(), m_y(), m_model() {}

ComCtrl::~ComCtrl() = default;

double ComCtrl::computeDesiredZetaSqr(const zVec3D* ref_com_position) const {
  return m_model.computeZetaSqr(ref_com_position);
}

double ComCtrl::computeDesiredZeta(const zVec3D* ref_com_position) const {
  return m_model.computeZeta(ref_com_position);
}

zVec3D* ComCtrl::computeDesiredZmpPosition(const zVec3D* ref_com_position,
                                           const zVec3D* com_position,
                                           const zVec3D* com_velocity,
                                           zVec3D* desired_zmp_position) const {
  double xd = zVec3DElem(ref_com_position, zX);
  double x = zVec3DElem(com_position, zX);
  double vx = zVec3DElem(com_velocity, zX);
  double yd = zVec3DElem(ref_com_position, zY);
  double y = zVec3DElem(com_position, zY);
  double vy = zVec3DElem(com_velocity, zY);
  double zeta = computeDesiredZeta(ref_com_position);
  zVec3DCreate(desired_zmp_position,
               m_x.computeDesiredZmpPosition(xd, x, vx, zeta),
               m_y.computeDesiredZmpPosition(yd, y, vy, zeta), 0);
  return desired_zmp_position;
}

}  // namespace holon
