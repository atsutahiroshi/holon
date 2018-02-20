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

#include <memory>

namespace holon {

ComCtrl::ComCtrl() : m_x(), m_y(), m_model() {
  m_user_cmds = std::make_shared<ComCtrlCommands>();
}

ComCtrl& ComCtrl::set_time_step(double t_time_step) {
  m_model.set_time_step(t_time_step);
  return *this;
}

Vec3D ComCtrl::computeDesZmpPos(const Vec3D& t_ref_com_pos,
                                const Vec3D& t_com_pos, const Vec3D& t_com_vel,
                                double t_desired_zeta) const {
  double xz =
      m_x.computeDesZmpPos(t_ref_com_pos, t_com_pos, t_com_vel, t_desired_zeta);
  double yz =
      m_y.computeDesZmpPos(t_ref_com_pos, t_com_pos, t_com_vel, t_desired_zeta);
  double zz = 0;
  return Vec3D(xz, yz, zz);
}

void ComCtrl::remapUserCommandsToInputs() {
  Vec3D default_com_pos(0, 0, m_model.data()->com_position.z());
  m_inputs.com_position = m_user_cmds->com_position.value_or(default_com_pos);
}

bool ComCtrl::update() {
  remapUserCommandsToInputs();
  m_outputs.zeta =
      m_model.computeZeta(m_inputs.com_position, kVec3DZero, kVec3DZero);
  if (zIsTiny(m_outputs.zeta)) return false;

  m_outputs.zmp_position =
      computeDesZmpPos(m_inputs.com_position, m_model.data()->com_position,
                       m_model.data()->com_velocity, m_outputs.zeta);
  m_model.data()->zmp_position = m_outputs.zmp_position;
  return m_model.update();
}

bool ComCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
