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

#include <roki/rk_g.h>
#include <memory>

namespace holon {

void ComCtrlCommands::clear() {
  xd = nullopt;
  yd = nullopt;
  zd = nullopt;
  vxd = nullopt;
  vyd = nullopt;
  qx1 = nullopt;
  qx2 = nullopt;
  qy1 = nullopt;
  qy2 = nullopt;
  qz1 = nullopt;
  qz2 = nullopt;
  vhp = nullopt;
}

void ComCtrlCommands::set_com_position(const Vec3D& t_com_position) {
  xd = t_com_position.x();
  yd = t_com_position.y();
  zd = t_com_position.z();
}

void ComCtrlCommands::set_com_position(optional<double> t_xd,
                                       optional<double> t_yd,
                                       optional<double> t_zd) {
  xd = t_xd;
  yd = t_yd;
  zd = t_zd;
}

void ComCtrlCommands::set_com_velocity(optional<double> t_vxd,
                                       optional<double> t_vyd) {
  vxd = t_vxd;
  vyd = t_vyd;
}

std::shared_ptr<ComCtrlCommands> ComCtrlCommandsFactory() {
  return std::make_shared<ComCtrlCommands>();
}

std::shared_ptr<ComCtrlInputs> ComCtrlInputsFactory() {
  return std::make_shared<ComCtrlInputs>();
}

std::shared_ptr<ComCtrlOutputs> ComCtrlOutputsFactory() {
  return std::make_shared<ComCtrlOutputs>();
}

ComCtrl::ComCtrl()
    : m_x(),
      m_y(),
      m_z(),
      m_model(),
      m_states_ptr(m_model.data_ptr()),
      m_inputs_ptr(ComCtrlInputsFactory()),
      m_outputs_ptr(ComCtrlOutputsFactory()),
      m_commands_ptr(ComCtrlCommandsFactory()),
      m_initial_com_position(m_states_ptr->com_position),
      m_initial_foot_dist(m_y.dist()) {}

ComCtrl::ComCtrl(const Model& t_model) : ComCtrl() {
  m_model.copy_data(t_model);
  m_initial_com_position = m_states_ptr->com_position;
}

ComCtrl& ComCtrl::set_states_ptr(StatesPtr t_states_ptr) {
  m_states_ptr = t_states_ptr;
  m_model.set_data_ptr(t_states_ptr);
  return *this;
}

ComCtrl& ComCtrl::set_inputs_ptr(InputsPtr t_inputs_ptr) {
  m_inputs_ptr = t_inputs_ptr;
  return *this;
}

ComCtrl& ComCtrl::set_outputs_ptr(OutputsPtr t_outputs_ptr) {
  m_outputs_ptr = t_outputs_ptr;
  return *this;
}

ComCtrl& ComCtrl::set_initial_com_position(const Vec3D& t_com_position) {
  m_initial_com_position = t_com_position;
  return *this;
}

ComCtrl& ComCtrl::set_initial_foot_dist(double t_initial_foot_dist) {
  m_initial_foot_dist = t_initial_foot_dist;
  return *this;
}

ComCtrl& ComCtrl::set_time_step(double t_time_step) {
  m_model.set_time_step(t_time_step);
  return *this;
}

ComCtrl& ComCtrl::reset(const Vec3D& t_com_position) {
  m_initial_com_position = t_com_position;
  m_model.reset(m_initial_com_position);
  return *this;
}

double ComCtrl::computeDesVrtReactForce(double t_zd, double t_z, double t_vz,
                                        double t_mass) const {
  return m_z.computeDesReactForce(t_zd, t_z, t_vz, t_mass);
}

double ComCtrl::computeDesZeta(double t_z, double t_zz, double t_fz,
                               double t_mass) const {
  return m_model.computeZeta(t_z, t_zz, t_fz, t_mass);
}

ComCtrl::HrzPos ComCtrl::computeDesHrzZmpPos(const Vec3D& t_ref_com_position,
                                             const Vec3D& t_com_position,
                                             const Vec3D& t_com_velocity,
                                             double t_desired_zeta) const {
  double xz = m_x.computeDesZmpPos(t_ref_com_position, t_com_position,
                                   t_com_velocity, t_desired_zeta);
  double yz = m_y.computeDesZmpPos(t_ref_com_position, t_com_position,
                                   t_com_velocity, t_desired_zeta);
  return std::make_tuple(xz, yz);
}

void ComCtrl::feedback(const Model& t_model) { feedback(t_model.data_ptr()); }

void ComCtrl::feedback(const Model::DataPtr& t_data_ptr) {
  feedback(t_data_ptr->com_position, t_data_ptr->com_velocity);
}

void ComCtrl::feedback(const Vec3D& t_com_position,
                       const Vec3D& t_com_velocity) {
  m_states_ptr->com_position = t_com_position;
  m_states_ptr->com_velocity = t_com_velocity;
}

void ComCtrl::remapCommandsToInputs() {
  m_inputs_ptr->com_position[0] =
      commands().xd.value_or(m_initial_com_position.x());
  m_inputs_ptr->com_position[1] =
      commands().yd.value_or(m_initial_com_position.y());
  m_inputs_ptr->com_position[2] =
      commands().zd.value_or(m_initial_com_position.z());
  m_inputs_ptr->com_velocity[0] = commands().vxd.value_or(0);
  m_inputs_ptr->com_velocity[1] = commands().vyd.value_or(0);
  m_inputs_ptr->com_velocity[2] = 0;
  m_inputs_ptr->qx1 = commands().qx1.value_or(ComCtrlX::default_q1);
  m_inputs_ptr->qx2 = commands().qx2.value_or(ComCtrlX::default_q2);
  m_inputs_ptr->qy1 = commands().qy1.value_or(ComCtrlY::default_q1);
  m_inputs_ptr->qy2 = commands().qy2.value_or(ComCtrlY::default_q2);
  m_inputs_ptr->qz1 = commands().qz1.value_or(ComCtrlZ::default_q1);
  m_inputs_ptr->qz2 = commands().qz2.value_or(ComCtrlZ::default_q2);
  m_inputs_ptr->vhp = commands().vhp.value_or(0);
}

void ComCtrl::updateCtrlParam() {
  x().set_q1(inputs().qx1);
  x().set_q2(inputs().qx2);
  y().set_q1(inputs().qy1);
  y().set_q2(inputs().qy2);
  z().set_q1(inputs().qz1);
  z().set_q2(inputs().qz2);
}

bool ComCtrl::update() {
  // remap commanded values given by user to referential values for controller
  remapCommandsToInputs();

  // update control parameters
  updateCtrlParam();

  // compute desired vertical reaction force
  double fz = computeDesVrtReactForce(
      inputs().com_position.z(), states().com_position.z(),
      states().com_velocity.z(), model().mass());

  // compute desired value of zeta from desired reaction force
  double zeta = computeDesZeta(states().com_position.z(), inputs().vhp, fz,
                               model().mass());
  // m_outputs_ptr->zeta = computeDesZeta(m_outputs_ptr->reaction_force);
  if (zIsTiny(zeta)) return false;

  // compute desired ZMP position
  double xz, yz;
  std::tie(xz, yz) =
      computeDesHrzZmpPos(inputs().com_position, states().com_position,
                          states().com_velocity, zeta);

  // update states of COM-ZMP model
  m_model.inputZmpPos(Vec3D(xz, yz, inputs().vhp), fz);
  if (!m_model.update()) return false;

  // update outputs of the controller
  m_outputs_ptr->zeta = zeta;
  m_outputs_ptr->com_position = states().com_position;
  m_outputs_ptr->com_velocity = states().com_velocity;
  m_outputs_ptr->com_acceleration = states().com_acceleration;
  m_outputs_ptr->zmp_position = states().zmp_position;
  m_outputs_ptr->reaction_force = states().reaction_force;
  return true;
}

bool ComCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
