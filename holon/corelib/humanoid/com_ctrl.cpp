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
#include "holon/corelib/humanoid/com_ctrl/com_ctrl_x.hpp"
#include "holon/corelib/humanoid/com_ctrl/com_ctrl_y.hpp"
#include "holon/corelib/humanoid/com_ctrl/com_ctrl_z.hpp"
#include "holon/corelib/humanoid/com_ctrl/phase_y.hpp"
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"
#include "holon/corelib/math/misc.hpp"

namespace holon {

namespace ctrl_x = com_ctrl_x;
namespace ctrl_y = com_ctrl_y;
namespace ctrl_z = com_ctrl_z;
namespace formula = com_zmp_model_formula;

void ComCtrlCommandsRawData::clear() {
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

void ComCtrlCommandsRawData::set_com_position(const Vec3D& t_com_position) {
  xd = t_com_position.x();
  yd = t_com_position.y();
  zd = t_com_position.z();
}

void ComCtrlCommandsRawData::set_com_position(optional<double> t_xd,
                                              optional<double> t_yd,
                                              optional<double> t_zd) {
  xd = t_xd;
  yd = t_yd;
  zd = t_zd;
}

void ComCtrlCommandsRawData::set_com_velocity(optional<double> t_vxd,
                                              optional<double> t_vyd) {
  vxd = t_vxd;
  vyd = t_vyd;
}

ComCtrlData::ComCtrlData(const Vec3D& t_com_position, double t_mass)
    : Base(ComZmpModelRawData{t_mass,
                              kVec3DZ,
                              t_com_position,
                              kVec3DZero,
                              kVec3DZero,
                              kVec3DZero,
                              {0, 0, t_mass * RK_G},
                              kVec3DZero,
                              {0, 0, t_mass * RK_G}},
           ComCtrlRefsRawData{ComZmpModelData::default_com_position, kVec3DZero,
                              ctrl_x::default_q1, ctrl_x::default_q2,
                              ctrl_y::default_q1, ctrl_y::default_q2,
                              ctrl_z::default_q1, ctrl_z::default_q2,
                              ctrl_y::default_rho, ctrl_y::default_dist,
                              ctrl_y::default_kr, 0},
           ComCtrlOutputsRawData(), ComCtrlCommandsRawData()) {
  {}
}

ComCtrl::ComCtrl() : ComCtrl(Base::model()) {}
ComCtrl::ComCtrl(const Model& t_model)
    : CtrlBase(t_model),
      m_default_com_position(model().initial_com_position()),
      m_canonical_foot_dist(ctrl_y::default_dist) {
  model().set_initial_com_position(states().com_position);
  m_default_com_position = model().initial_com_position();
  model().setReactionForceCallback(getReactionForceCallback());
  model().setZmpPositionCallback(getZmpPositionCallback());
}

ComCtrl& ComCtrl::set_canonical_foot_dist(double t_canonical_foot_dist) {
  m_canonical_foot_dist = t_canonical_foot_dist;
  return *this;
}

ComCtrl& ComCtrl::reset() {
  model().reset();
  return *this;
}

ComCtrl& ComCtrl::reset(const Vec3D& t_com_position) {
  model().reset(t_com_position);
  m_default_com_position = model().initial_com_position();
  return *this;
}

ComCtrl& ComCtrl::reset(const Vec3D& t_com_position, double t_foot_dist) {
  m_canonical_foot_dist = t_foot_dist;
  m_current_foot_dist = t_foot_dist;
  refs().dist = t_foot_dist;
  return reset(t_com_position);
}

void ComCtrl::feedback(const Model& t_model) { feedback(t_model.data()); }

void ComCtrl::feedback(ComZmpModelData t_model_data) {
  feedback(t_model_data.get<0>().com_position,
           t_model_data.get<0>().com_velocity);
}

void ComCtrl::feedback(const Vec3D& t_com_position,
                       const Vec3D& t_com_velocity) {
  states().com_position = t_com_position;
  states().com_velocity = t_com_velocity;
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
                                     refs().com_position, refs().com_velocity,
                                     refs().qx1, refs().qx2, zeta);
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

double ComCtrl::phaseLF() const {
  auto zeta =
      formula::computeZeta(outputs().com_position, outputs().zmp_position,
                           outputs().com_acceleration);
  auto pz = phase_y::computeComplexZmp(
      outputs().zmp_position, outputs().com_velocity, refs().com_position,
      refs().qy1, refs().qy2, zeta);
  auto pLin = phase_y::computeComplexInnerEdge(refs().com_position,
                                               refs().com_position, pz, 1);
  return phase_y::computePhase(pz, pLin);
}

double ComCtrl::phaseRF() const {
  auto zeta =
      formula::computeZeta(outputs().com_position, outputs().zmp_position,
                           outputs().com_acceleration);
  auto pz = phase_y::computeComplexZmp(
      outputs().zmp_position, outputs().com_velocity, refs().com_position,
      refs().qy1, refs().qy2, zeta);
  auto pRin = phase_y::computeComplexInnerEdge(refs().com_position,
                                               refs().com_position, pz, -1);
  return phase_y::computePhase(pz, pRin);
}

void ComCtrl::updateSideward() {
  auto zeta = formula::computeZeta(states().com_position, states().zmp_position,
                                   states().com_acceleration);
  auto pz = phase_y::computeComplexZmp(
      outputs().zmp_position, outputs().com_velocity, refs().com_position,
      refs().qy1, refs().qy2, zeta);
  auto pLin = phase_y::computeComplexInnerEdge(refs().com_position,
                                               refs().com_position, pz, 1);
  auto pRin = phase_y::computeComplexInnerEdge(refs().com_position,
                                               refs().com_position, pz, -1);
  auto phaseL = phase_y::computePhase(pz, pLin);
  auto phaseR = phase_y::computePhase(pz, pRin);
  double yd = refs().com_position[1];
  double vyd = refs().com_velocity[1];
  double T = phase_y::computePeriod(refs().qy1, refs().qy2, zeta);
  double d0 = canonical_foot_dist();
  double d = m_current_foot_dist;
  if ((vyd > 0 && phaseR > 0 && phaseR < 1) ||
      (vyd < 0 && phaseL > 0 && phaseL < 1)) {
    // following phase
    double phase;
    if (vyd > 0)
      phase = phaseR;
    else
      phase = phaseL;
    double d_new = d0 + T * phase * std::fabs(vyd);
    double yd_new = yd + 0.5 * (d_new - d) * sgn(vyd);
    refs().com_position[1] = yd_new;
    refs().dist = d_new;
    m_max_foot_dist = refs().dist;
  } else if ((vyd > 0 && phaseL > 0 && phaseL < 1) ||
             (vyd < 0 && phaseR > 0 && phaseR < 1)) {
    // braking phase
    double phase;
    if (vyd > 0)
      phase = phaseL;
    else
      phase = phaseR;
    double d_new = m_max_foot_dist + (d0 - m_max_foot_dist) * phase;
    double yd_new = yd + 0.5 * (d - d_new) * sgn(vyd);
    refs().com_position[1] = yd_new;
    refs().dist = d_new;
  } else {
    // double-supporting
    // do nothing
  }
}

void ComCtrl::updateRefs() {
  refs().com_position[0] = commands().xd.value_or(default_com_position().x());
  refs().com_position[1] = commands().yd.value_or(default_com_position().y());
  refs().com_position[2] = commands().zd.value_or(default_com_position().z());
  refs().com_velocity[0] = commands().vxd.value_or(0);
  refs().com_velocity[1] = commands().vyd.value_or(0);
  refs().com_velocity[2] = 0;
  refs().rho = commands().rho.value_or(ctrl_y::default_rho);
  refs().qx1 = commands().qx1.value_or(ctrl_x::default_q1);
  if (commands().dist) set_canonical_foot_dist(commands().dist.value());
  refs().dist = commands().dist.value_or(m_canonical_foot_dist);
  if (!zIsTiny(refs().com_velocity[0]) || !zIsTiny(refs().com_velocity[1])) {
    refs().rho = 1;
    if (!zIsTiny(refs().com_velocity[0])) {
      refs().qx1 = 0;
    }
    if (!zIsTiny(refs().com_velocity[1])) {
      updateSideward();
    }
  }
  refs().qx2 = commands().qx2.value_or(ctrl_x::default_q2);
  refs().qy1 = commands().qy1.value_or(ctrl_y::default_q1);
  refs().qy2 = commands().qy2.value_or(ctrl_y::default_q2);
  refs().kr = commands().kr.value_or(ctrl_y::default_kr);
  refs().qz1 = commands().qz1.value_or(ctrl_z::default_q1);
  refs().qz2 = commands().qz2.value_or(ctrl_z::default_q2);
  refs().vhp = commands().vhp.value_or(0);
}

void ComCtrl::updateOutputs() {
  outputs().com_position = states().com_position;
  outputs().com_velocity = states().com_velocity;
  outputs().com_acceleration = states().com_acceleration;
  outputs().zmp_position = states().zmp_position;
  outputs().reaction_force = states().reaction_force;
}

void ComCtrl::updateDefaultComPosition() {
  if (commands().vxd && !zIsTiny(commands().vxd.value())) {
    m_default_com_position[0] = states().com_position.x();
  }
  if (commands().vyd && !zIsTiny(commands().vyd.value())) {
    m_default_com_position[1] = refs().com_position.y();
    m_current_foot_dist = refs().dist;
  }
  if (commands().zd) {
    m_default_com_position[2] = commands().zd.value();
  }
}

bool ComCtrl::update() {
  updateRefs();
  if (!model().update()) return false;
  updateOutputs();
  updateDefaultComPosition();
  return true;
}

bool ComCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

const double ComCtrlData::default_mass = ComZmpModelData::default_mass;
const Vec3D ComCtrlData::default_com_position =
    ComZmpModelData::default_com_position;

}  // namespace holon
