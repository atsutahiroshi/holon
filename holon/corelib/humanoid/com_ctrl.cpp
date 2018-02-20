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

ComCtrl::ComCtrl()
    : m_x(),
      m_y(),
      m_model(),
      m_states_ptr(m_model.data()),
      m_inputs_ptr(std::make_shared<ComCtrlInputs>()),
      m_outputs_ptr(std::make_shared<ComCtrlOutputs>()),
      m_user_cmds_ptr(std::make_shared<ComCtrlCommands>()),
      m_initial_com_position(m_states_ptr->com_position) {}

ComCtrl& ComCtrl::set_time_step(double t_time_step) {
  m_model.set_time_step(t_time_step);
  return *this;
}

ComCtrl& ComCtrl::reset(const Vec3D& t_com_position) {
  m_initial_com_position = t_com_position;
  m_model.reset(m_initial_com_position);
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
  m_inputs_ptr->com_position = cmds().com_position.value_or(default_com_pos);
}

bool ComCtrl::update() {
  // remap commanded values given by user to referential values for controller
  remapUserCommandsToInputs();

  // compute desired reaction force along z-axis
  // TODO: add computation of reaction force

  // compute desired value of zeta from desired reaction force
  // TODO: fix computation of zeta
  // TODO: check if COM position should be given from inputs (states?)
  m_outputs_ptr->zeta =
      m_model.computeZeta(inputs().com_position, kVec3DZero, kVec3DZero);
  if (zIsTiny(outputs().zeta)) return false;

  // compute desired ZMP position
  m_outputs_ptr->zmp_position =
      computeDesZmpPos(inputs().com_position, states().com_position,
                       states().com_velocity, outputs().zeta);

  // update states of COM-ZMP model
  // TODO: add reaction force
  m_states_ptr->zmp_position = outputs().zmp_position;

  // TODO: add update of desired position, velocity and acceleration of COM
  return m_model.update();
}

bool ComCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
