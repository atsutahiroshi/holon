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
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

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
  rho = nullopt;
  dist = nullopt;
  kr = nullopt;
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

ComCtrlInputs::ComCtrlInputs()
    : com_position(ComZmpModelData::default_com_position),
      com_velocity(kVec3DZero),
      qx1(ComCtrlX::default_q1),
      qx2(ComCtrlX::default_q2),
      qy1(ComCtrlY::default_q1),
      qy2(ComCtrlY::default_q2),
      qz1(ComCtrlZ::default_q1),
      qz2(ComCtrlZ::default_q2),
      rho(ComCtrlY::default_rho),
      dist(ComCtrlY::default_dist),
      kr(ComCtrlY::default_kr),
      vhp(0) {}

ComCtrlInputs::ComCtrlInputs(const ComZmpModelData& t_data)
    : com_position(t_data.com_position),
      com_velocity(kVec3DZero),
      qx1(ComCtrlX::default_q1),
      qx2(ComCtrlX::default_q2),
      qy1(ComCtrlY::default_q1),
      qy2(ComCtrlY::default_q2),
      qz1(ComCtrlZ::default_q1),
      qz2(ComCtrlZ::default_q2),
      rho(ComCtrlY::default_rho),
      dist(ComCtrlY::default_dist),
      kr(ComCtrlY::default_kr),
      vhp(0) {}

ComCtrlInputs::ComCtrlInputs(const ComZmpModel& t_model)
    : ComCtrlInputs(t_model.data()) {}

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
      m_canonical_foot_dist(m_y.dist()) {
  m_model.setReactionForceCallback(getReactionForceCallback());
  m_model.setZmpPositionCallback(getZmpPositionCallback());
}

ComCtrl::ComCtrl(const Model& t_model) : ComCtrl() {
  m_model.copy_data(t_model);
  m_model.set_initial_com_position(states().com_position);
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

ComCtrl& ComCtrl::set_canonical_foot_dist(double t_canonical_foot_dist) {
  m_canonical_foot_dist = t_canonical_foot_dist;
  return *this;
}

ComCtrl& ComCtrl::set_time_step(double t_time_step) {
  m_model.set_time_step(t_time_step);
  return *this;
}

ComCtrl& ComCtrl::reset(const Vec3D& t_com_position) {
  m_model.reset(t_com_position);
  return *this;
}

ComCtrl& ComCtrl::reset(const Vec3D& t_com_position, double t_foot_dist) {
  m_canonical_foot_dist = t_foot_dist;
  return reset(t_com_position);
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

Vec3D ComCtrl::computeDesReactForce(const Vec3D& t_com_position,
                                    const Vec3D& t_com_velocity,
                                    const double /* t */) {
  auto fz = m_z.computeDesReactForce(inputs().com_position, t_com_position,
                                     t_com_velocity, model().mass());
  return Vec3D(0, 0, fz);
}

Vec3D ComCtrl::computeDesZmpPos(const Vec3D& t_com_position,
                                const Vec3D& t_com_velocity,
                                const double /* t */) {
  auto fz = m_z.computeDesReactForce(inputs().com_position, t_com_position,
                                     t_com_velocity, model().mass());
  auto zeta = ComZmpModelFormula::computeZeta(t_com_position.z(), inputs().vhp,
                                              fz, model().mass());
  auto xz = m_x.computeDesZmpPos(inputs().com_position, t_com_position,
                                 t_com_velocity, zeta);
  auto yz = m_y.computeDesZmpPos(inputs().com_position, t_com_position,
                                 t_com_velocity, zeta);
  return Vec3D(xz, yz, inputs().vhp);
}

ComCtrl::CallbackFunc ComCtrl::getReactionForceCallback() {
  namespace pl = std::placeholders;
  return std::bind(&ComCtrl::computeDesReactForce, this, pl::_1, pl::_2,
                   pl::_3);
}

ComCtrl::CallbackFunc ComCtrl::getZmpPositionCallback() {
  namespace pl = std::placeholders;
  return std::bind(&ComCtrl::computeDesZmpPos, this, pl::_1, pl::_2, pl::_3);
}

void ComCtrl::remapCommandsToInputs() {
  m_inputs_ptr->com_position[0] =
      commands().xd.value_or(model().initial_com_position().x());
  m_inputs_ptr->com_position[1] =
      commands().yd.value_or(model().initial_com_position().y());
  m_inputs_ptr->com_position[2] =
      commands().zd.value_or(model().initial_com_position().z());
  m_inputs_ptr->com_velocity[0] = commands().vxd.value_or(0);
  m_inputs_ptr->com_velocity[1] = commands().vyd.value_or(0);
  m_inputs_ptr->com_velocity[2] = 0;
  m_inputs_ptr->qx1 = commands().qx1.value_or(ComCtrlX::default_q1);
  m_inputs_ptr->qx2 = commands().qx2.value_or(ComCtrlX::default_q2);
  m_inputs_ptr->qy1 = commands().qy1.value_or(ComCtrlY::default_q1);
  m_inputs_ptr->qy2 = commands().qy2.value_or(ComCtrlY::default_q2);
  m_inputs_ptr->rho = commands().rho.value_or(ComCtrlY::default_rho);
  m_inputs_ptr->dist = commands().dist.value_or(m_canonical_foot_dist);
  m_inputs_ptr->kr = commands().kr.value_or(ComCtrlY::default_kr);
  m_inputs_ptr->qz1 = commands().qz1.value_or(ComCtrlZ::default_q1);
  m_inputs_ptr->qz2 = commands().qz2.value_or(ComCtrlZ::default_q2);
  m_inputs_ptr->vhp = commands().vhp.value_or(0);
}

void ComCtrl::updateCtrlParam() {
  x().set_q1(inputs().qx1);
  x().set_q2(inputs().qx2);
  y().set_q1(inputs().qy1);
  y().set_q2(inputs().qy2);
  y().set_rho(inputs().rho);
  y().set_dist(inputs().dist);
  y().set_kr(inputs().kr);
  z().set_q1(inputs().qz1);
  z().set_q2(inputs().qz2);
}

void ComCtrl::updateOutputs() {
  m_outputs_ptr->com_position = states().com_position;
  m_outputs_ptr->com_velocity = states().com_velocity;
  m_outputs_ptr->com_acceleration = states().com_acceleration;
  m_outputs_ptr->zmp_position = states().zmp_position;
  m_outputs_ptr->reaction_force = states().reaction_force;
}

bool ComCtrl::update() {
  remapCommandsToInputs();
  updateCtrlParam();
  if (!m_model.update()) return false;
  updateOutputs();
  return true;
}

bool ComCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
