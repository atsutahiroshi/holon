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

ComCtrl::ComCtrl() : m_x(), m_y(), m_model() {
  zVec3DCopy(model().com_position(), &m_cmd_com_position);
  zVec3DClear(&m_des_zmp_position);
}

ComCtrl::~ComCtrl() = default;

ComCtrl& ComCtrl::set_time_step(double t_time_step) {
  m_model.set_time_step(t_time_step);
  return *this;
}

ComCtrl& ComCtrl::set_cmd_com_position(const zVec3D* t_cmd_com_position) {
  zVec3DCopy(t_cmd_com_position, &m_cmd_com_position);
  return *this;
}

double ComCtrl::computeDesiredZetaSqr(const zVec3D* ref_com_position) const {
  return m_model.computeZetaSqr(ref_com_position);
}

double ComCtrl::computeDesiredZeta(const zVec3D* ref_com_position) const {
  return m_model.computeZeta(ref_com_position);
}

zVec3D* ComCtrl::computeDesiredZmpPosition(const zVec3D* ref_com_pos,
                                           const zVec3D* com_pos,
                                           const zVec3D* com_vel,
                                           zVec3D* desired_zmp_pos) const {
  double zeta = computeDesiredZeta(ref_com_pos);
  zVec3DCreate(
      desired_zmp_pos,
      m_x.computeDesiredZmpPosition(ref_com_pos, com_pos, com_vel, zeta),
      m_y.computeDesiredZmpPosition(ref_com_pos, com_pos, com_vel, zeta), 0);
  return desired_zmp_pos;
}

}  // namespace holon
