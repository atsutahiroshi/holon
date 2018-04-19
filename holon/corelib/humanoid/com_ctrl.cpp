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
#include "holon/corelib/humanoid/biped_foot_model.hpp"
#include "holon/corelib/humanoid/com_ctrl/com_guided_ctrl_formula.hpp"
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"
#include "holon/corelib/math/misc.hpp"

namespace holon {

namespace cz = com_zmp_model_formula;
namespace cgc = com_guided_ctrl_formula;

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

const Array3d ComCtrlParamsRawData::default_q1 = {{1, 1, 1}};
const Array3d ComCtrlParamsRawData::default_q2 = {{1, 1, 1}};
const double ComCtrlParamsRawData::default_rho = 0;
const double ComCtrlParamsRawData::default_dist = 0;
const double ComCtrlParamsRawData::default_kr = 1;
const double ComCtrlParamsRawData::default_vhp = 0;

ComCtrlParamsRawData::ComCtrlParamsRawData()
    : ComCtrlParamsRawData(ComZmpModelRawData::default_com_position,
                           ComZmpModelRawData::default_mass) {}

ComCtrlParamsRawData::ComCtrlParamsRawData(const Vec3D& t_com_position,
                                           double t_mass)
    : com_position(t_com_position),
      com_velocity(kVec3DZero),
      mass(t_mass),
      q1(ComCtrlParamsRawData::default_q1),
      q2(ComCtrlParamsRawData::default_q1),
      rho(ComCtrlParamsRawData::default_rho),
      dist(ComCtrlParamsRawData::default_dist),
      kr(ComCtrlParamsRawData::default_kr),
      vhp(ComCtrlParamsRawData::default_vhp) {}

ComCtrlOutputsRawData::ComCtrlOutputsRawData()
    : com_position(kVec3DZero),
      com_velocity(kVec3DZero),
      com_acceleration(kVec3DZero),
      zmp_position(kVec3DZero),
      reaction_force(kVec3DZero) {}

ComCtrlData::ComCtrlData(const Vec3D& t_com_position, double t_mass)
    : DataSetBase(alloc_raw_data<ComZmpModelRawData>(t_mass, t_com_position),
                  alloc_raw_data<ComCtrlParamsRawData>(t_com_position, t_mass),
                  alloc_raw_data<ComCtrlOutputsRawData>(),
                  alloc_raw_data<ComCtrlCommandsRawData>()) {}

ComCtrl::ComCtrl() : ComCtrl(make_data<Data>()) {}
ComCtrl::ComCtrl(const Model& t_model)
    : CtrlBase(t_model),
      m_canonical_foot_dist(ComCtrlParamsRawData::default_dist) {
  init();
}
ComCtrl::ComCtrl(Data t_data)
    : CtrlBase(t_data),
      m_canonical_foot_dist(ComCtrlParamsRawData::default_dist) {
  init();
}
ComCtrl::ComCtrl(Data t_data, std::shared_ptr<Model> t_model_ptr)
    : CtrlBase(t_data, t_model_ptr),
      m_canonical_foot_dist(ComCtrlParamsRawData::default_dist) {
  init();
}

void ComCtrl::init() {
  params().mass = model().mass();
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
  params().dist = t_foot_dist;
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
  auto fz =
      cgc::desired_reaction_force_z(t_com_position, t_com_velocity, params());
  return Vec3D(0, 0, fz);
}

Vec3D ComCtrl::computeDesZmpPos(const Vec3D& t_com_position,
                                const Vec3D& t_com_velocity,
                                const double /* t */) {
  using namespace index_symbols;
  auto fz =
      cgc::desired_reaction_force_z(t_com_position, t_com_velocity, params());
  auto zeta = cz::zeta(t_com_position[_Z], params().vhp, fz, model().mass());
  return cgc::desired_zmp_position(t_com_position, t_com_velocity, params(),
                                   zeta);
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
  auto zeta = cz::zeta(outputs().com_position, outputs().zmp_position,
                       outputs().com_acceleration);
  auto pz = cgc::complex_zmp_y(outputs().zmp_position, outputs().com_velocity,
                               params(), zeta);
  auto pLin = cgc::complex_inner_edge(params().com_position, params(), pz,
                                      BipedFootType::left);
  return cgc::phase_y(pz, pLin);
}

double ComCtrl::phaseRF() const {
  auto zeta = cz::zeta(outputs().com_position, outputs().zmp_position,
                       outputs().com_acceleration);
  auto pz = cgc::complex_zmp_y(outputs().zmp_position, outputs().com_velocity,
                               params(), zeta);
  auto pRin = cgc::complex_inner_edge(params().com_position, params(), pz,
                                      BipedFootType::right);
  return cgc::phase_y(pz, pRin);
}

void ComCtrl::updateSideward() {
  using namespace index_symbols;
  auto zeta = cz::zeta(states().com_position, states().zmp_position,
                       states().com_acceleration);
  auto pz = cgc::complex_zmp_y(outputs().zmp_position, outputs().com_velocity,
                               params(), zeta);
  auto pLin = cgc::complex_inner_edge(params().com_position, params(), pz,
                                      BipedFootType::left);
  auto pRin = cgc::complex_inner_edge(params().com_position, params(), pz,
                                      BipedFootType::right);
  auto phaseL = cgc::phase_y(pz, pLin);
  auto phaseR = cgc::phase_y(pz, pRin);
  double yd = params().com_position[1];
  double vyd = params().com_velocity[1];
  double T = cgc::period(params().q1[_Y], params().q2[_Y], zeta);
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
    params().com_position[1] = yd_new;
    params().dist = d_new;
    m_max_foot_dist = params().dist;
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
    params().com_position[1] = yd_new;
    params().dist = d_new;
  } else {
    // double-supporting
    // do nothing
  }
}

void ComCtrl::updateParams() {
  using namespace index_symbols;
  params().com_position[0] = commands().xd.value_or(default_com_position().x());
  params().com_position[1] = commands().yd.value_or(default_com_position().y());
  params().com_position[2] = commands().zd.value_or(default_com_position().z());
  params().com_velocity[0] = commands().vxd.value_or(0);
  params().com_velocity[1] = commands().vyd.value_or(0);
  params().com_velocity[2] = 0;
  params().rho = commands().rho.value_or(ComCtrlParamsRawData::default_rho);
  params().q1[_X] =
      commands().qx1.value_or(ComCtrlParamsRawData::default_q1[_X]);
  if (commands().dist) set_canonical_foot_dist(commands().dist.value());
  params().dist = commands().dist.value_or(m_canonical_foot_dist);
  if (!zIsTiny(params().com_velocity[0]) ||
      !zIsTiny(params().com_velocity[1])) {
    params().rho = 1;
    if (!zIsTiny(params().com_velocity[0])) {
      params().q1[_X] = 0;
    }
    if (!zIsTiny(params().com_velocity[1])) {
      updateSideward();
    }
  }
  params().q2[_X] =
      commands().qx2.value_or(ComCtrlParamsRawData::default_q2[_X]);
  params().q1[_Y] =
      commands().qy1.value_or(ComCtrlParamsRawData::default_q1[_Y]);
  params().q2[_Y] =
      commands().qy2.value_or(ComCtrlParamsRawData::default_q2[_Y]);
  params().kr = commands().kr.value_or(ComCtrlParamsRawData::default_kr);
  params().q1[_Z] =
      commands().qz1.value_or(ComCtrlParamsRawData::default_q1[_Z]);
  params().q2[_Z] =
      commands().qz2.value_or(ComCtrlParamsRawData::default_q2[_Z]);
  params().vhp = commands().vhp.value_or(0);
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
    m_default_com_position[1] = params().com_position.y();
    m_current_foot_dist = params().dist;
  }
  if (commands().zd) {
    m_default_com_position[2] = commands().zd.value();
  }
}

bool ComCtrl::update() {
  updateParams();
  if (!model().update()) return false;
  updateOutputs();
  updateDefaultComPosition();
  return true;
}

bool ComCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
