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
#include "holon/corelib/humanoid/com_ctrl_x.hpp"
#include "holon/corelib/humanoid/com_ctrl_y.hpp"
#include "holon/corelib/humanoid/com_ctrl_z.hpp"
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

namespace holon {

namespace ctrl_x = com_ctrl_x;
namespace ctrl_y = com_ctrl_y;
namespace ctrl_z = com_ctrl_z;
namespace formula = ComZmpModelFormula;

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

ComCtrlRefs::ComCtrlRefs()
    : com_position(ComZmpModelData::default_com_position),
      com_velocity(kVec3DZero),
      qx1(ctrl_x::default_q1),
      qx2(ctrl_x::default_q2),
      qy1(ctrl_y::default_q1),
      qy2(ctrl_y::default_q2),
      qz1(ctrl_z::default_q1),
      qz2(ctrl_z::default_q2),
      rho(ctrl_y::default_rho),
      dist(ctrl_y::default_dist),
      kr(ctrl_y::default_kr),
      vhp(0) {}

ComCtrlRefs::ComCtrlRefs(const ComZmpModelData& t_data)
    : com_position(t_data.com_position),
      com_velocity(kVec3DZero),
      qx1(ctrl_x::default_q1),
      qx2(ctrl_x::default_q2),
      qy1(ctrl_y::default_q1),
      qy2(ctrl_y::default_q2),
      qz1(ctrl_z::default_q1),
      qz2(ctrl_z::default_q2),
      rho(ctrl_y::default_rho),
      dist(ctrl_y::default_dist),
      kr(ctrl_y::default_kr),
      vhp(0) {}

ComCtrlRefs::ComCtrlRefs(const ComZmpModel& t_model)
    : ComCtrlRefs(t_model.data()) {}

ComCtrlCommandsPtr createComCtrlCommands() {
  return std::make_shared<ComCtrlCommands>();
}

ComCtrlRefsPtr createComCtrlRefs() { return std::make_shared<ComCtrlRefs>(); }

ComCtrlOutputsPtr createComCtrlOutputs() {
  return std::make_shared<ComCtrlOutputs>();
}

ComCtrl::ComCtrl()
    : m_model(),
      m_states_ptr(m_model.data_ptr()),
      m_refs_ptr(createComCtrlRefs()),
      m_outputs_ptr(createComCtrlOutputs()),
      m_commands_ptr(createComCtrlCommands()),
      m_canonical_foot_dist(ctrl_y::default_dist) {
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

ComCtrl& ComCtrl::set_refs_ptr(RefsPtr t_refs_ptr) {
  m_refs_ptr = t_refs_ptr;
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
  auto fz = ctrl_z::computeDesReactForce(t_com_position, t_com_velocity,
                                         refs().com_position, refs().qz1,
                                         refs().qz2, model().mass());
  return Vec3D(0, 0, fz);
}

Vec3D ComCtrl::computeDesZmpPos(const Vec3D& t_com_position,
                                const Vec3D& t_com_velocity,
                                const double /* t */) {
  auto fz = ctrl_z::computeDesReactForce(t_com_position, t_com_velocity,
                                         refs().com_position, refs().qz1,
                                         refs().qz2, model().mass());
  auto zeta =
      formula::computeZeta(t_com_position.z(), refs().vhp, fz, model().mass());
  auto xz = ctrl_x::computeDesZmpPos(t_com_position, t_com_velocity,
                                     refs().com_position, refs().qx1,
                                     refs().qx2, zeta);
  auto yz = ctrl_y::computeDesZmpPos(
      t_com_position, t_com_velocity, refs().com_position, refs().qy1,
      refs().qy2, refs().rho, refs().dist, refs().kr, zeta);
  return Vec3D(xz, yz, refs().vhp);
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

void ComCtrl::remapCommandsToRefs() {
  m_refs_ptr->com_position[0] =
      commands().xd.value_or(model().initial_com_position().x());
  m_refs_ptr->com_position[1] =
      commands().yd.value_or(model().initial_com_position().y());
  m_refs_ptr->com_position[2] =
      commands().zd.value_or(model().initial_com_position().z());
  m_refs_ptr->com_velocity[0] = commands().vxd.value_or(0);
  m_refs_ptr->com_velocity[1] = commands().vyd.value_or(0);
  m_refs_ptr->com_velocity[2] = 0;
  m_refs_ptr->qx1 = commands().qx1.value_or(ctrl_x::default_q1);
  m_refs_ptr->qx2 = commands().qx2.value_or(ctrl_x::default_q2);
  m_refs_ptr->qy1 = commands().qy1.value_or(ctrl_y::default_q1);
  m_refs_ptr->qy2 = commands().qy2.value_or(ctrl_y::default_q2);
  m_refs_ptr->rho = commands().rho.value_or(ctrl_y::default_rho);
  m_refs_ptr->dist = commands().dist.value_or(m_canonical_foot_dist);
  m_refs_ptr->kr = commands().kr.value_or(ctrl_y::default_kr);
  m_refs_ptr->qz1 = commands().qz1.value_or(ctrl_z::default_q1);
  m_refs_ptr->qz2 = commands().qz2.value_or(ctrl_z::default_q2);
  m_refs_ptr->vhp = commands().vhp.value_or(0);
}

void ComCtrl::updateOutputs() {
  m_outputs_ptr->com_position = states().com_position;
  m_outputs_ptr->com_velocity = states().com_velocity;
  m_outputs_ptr->com_acceleration = states().com_acceleration;
  m_outputs_ptr->zmp_position = states().zmp_position;
  m_outputs_ptr->reaction_force = states().reaction_force;
}

bool ComCtrl::update() {
  remapCommandsToRefs();
  if (!m_model.update()) return false;
  updateOutputs();
  return true;
}

bool ComCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
