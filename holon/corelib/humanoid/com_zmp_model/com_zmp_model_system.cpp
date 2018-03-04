/* com_zmp_model_system - Definition of the COM-ZMP model system
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

#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_system.hpp"

#include <roki/rk_g.h>
#include <cassert>
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

namespace holon {

using com_zmp_model_formula::computeSqrZeta;
using com_zmp_model_formula::computeZeta;
using com_zmp_model_formula::computeReactForce;
using com_zmp_model_formula::computeComAcc;
using com_zmp_model_formula::isMassValid;
using com_zmp_model_formula::isComZmpDiffValid;
using com_zmp_model_formula::isReactionForceValid;
using com_zmp_model_formula::isComAccelerationValid;

ComZmpModelSystem::ComZmpModelSystem(ComZmpModelDataPtr t_data_ptr)
    : m_data_ptr(t_data_ptr),
      m_com_acceleration_f(getDefaultComAccFunc()),
      m_reaction_force_f(getDefaultReactForceFunc()),
      m_external_force_f(getDefaultExtForceFunc()),
      m_zmp_position_f(nullptr) {}

void ComZmpModelSystem::operator()(const State& x, State& dxdt,
                                   const double t) const {
  assert(m_com_acceleration_f);
  dxdt[0] = x[1];
  dxdt[1] = m_com_acceleration_f(x[0], x[1], t);
}

ComZmpModelSystem::Function ComZmpModelSystem::getDefaultComAccFunc() const {
  return
      [this](const Vec3D&, const Vec3D&, const double) { return kVec3DZero; };
}

ComZmpModelSystem::Function ComZmpModelSystem::getComAccFuncWithReactForce()
    const {
  return [this](const Vec3D& p, const Vec3D& v, const double t) {
    return computeComAcc(m_reaction_force_f(p, v, t), m_data_ptr->mass,
                         m_external_force_f(p, v, t));
  };
}
ComZmpModelSystem::Function ComZmpModelSystem::getComAccFuncWithZmpPos() const {
  return [this](const Vec3D& p, const Vec3D& v, const double t) {
    return computeComAcc(p, m_zmp_position_f(p, v, t),
                         m_reaction_force_f(p, v, t), m_data_ptr->mass,
                         m_external_force_f(p, v, t));
  };
}

ComZmpModelSystem::Function ComZmpModelSystem::getDefaultReactForceFunc()
    const {
  return [this](const Vec3D&, const Vec3D&, const double) {
    return Vec3D(0, 0, m_data_ptr->mass * RK_G);
  };
}

ComZmpModelSystem::Function ComZmpModelSystem::getDefaultExtForceFunc() const {
  return
      [this](const Vec3D&, const Vec3D&, const double) { return kVec3DZero; };
}

ComZmpModelSystem::Function ComZmpModelSystem::getDefaultZmpPosFunc() const {
  return nullptr;
}

ComZmpModelSystem::self_ref ComZmpModelSystem::set_data_ptr(
    DataPtr t_data_ptr) {
  m_data_ptr = t_data_ptr;
  return *this;
}

ComZmpModelSystem::self_ref ComZmpModelSystem::set_com_acceleration_f(
    Function t_com_acceleration_f) {
  if (t_com_acceleration_f)
    m_com_acceleration_f = t_com_acceleration_f;
  else
    m_com_acceleration_f = getDefaultComAccFunc();
  return *this;
}

ComZmpModelSystem::self_ref ComZmpModelSystem::set_reaction_force_f(
    Function t_reaction_force_f) {
  if (t_reaction_force_f)
    m_reaction_force_f = t_reaction_force_f;
  else
    m_reaction_force_f = getDefaultReactForceFunc();

  if (m_zmp_position_f)
    return set_com_acceleration_f(getComAccFuncWithZmpPos());
  else
    return set_com_acceleration_f(getComAccFuncWithReactForce());
}

ComZmpModelSystem::self_ref ComZmpModelSystem::set_external_force_f(
    Function t_external_force_f) {
  if (t_external_force_f)
    m_external_force_f = t_external_force_f;
  else
    m_external_force_f = getDefaultExtForceFunc();

  if (m_zmp_position_f)
    return set_com_acceleration_f(getComAccFuncWithZmpPos());
  else
    return set_com_acceleration_f(getComAccFuncWithReactForce());
}

ComZmpModelSystem::self_ref ComZmpModelSystem::set_zmp_position_f(
    Function t_zmp_position_f) {
  if (t_zmp_position_f)
    m_zmp_position_f = t_zmp_position_f;
  else
    m_zmp_position_f = getDefaultZmpPosFunc();

  if (m_zmp_position_f)
    return set_com_acceleration_f(getComAccFuncWithZmpPos());
  else
    return set_com_acceleration_f(getComAccFuncWithReactForce());
}

}  // namespace holon
