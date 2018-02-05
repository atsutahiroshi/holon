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
#include "holon/corelib/humanoid/com_zmp_model.hpp"

namespace holon {

ComCtrl::ComCtrl() : m_q1(default_q1), m_q2(default_q2) {}
ComCtrl::ComCtrl(double t_q1, double t_q2) : m_q1(t_q1), m_q2(t_q2) {}

ComCtrl::~ComCtrl() {}

ComCtrl& ComCtrl::set_q1(double t_q1) {
  m_q1 = t_q1;
  return *this;
}

ComCtrl& ComCtrl::set_q2(double t_q2) {
  m_q2 = t_q2;
  return *this;
}

double ComCtrl::ComputeDesiredZetaSqr(const zVec3D* ref_com_position) const {
  return ComZmpModel().ComputeZetaSqr(ref_com_position);
}

double ComCtrl::ComputeDesiredZeta(const zVec3D* ref_com_position) const {
  return ComZmpModel().ComputeZeta(ref_com_position);
}

zVec3D* ComCtrl::ComputeDesiredZmpPosition(const zVec3D* ref_com_position,
                                           const zVec3D* com_position,
                                           const zVec3D* com_velocity,
                                           zVec3D* desired_zmp_position) const {
  (void)ref_com_position;
  (void)com_position;
  (void)com_velocity;
  zVec3DCreate(desired_zmp_position, 0, 0, 0);
  return desired_zmp_position;
}

}  // namespace holon
