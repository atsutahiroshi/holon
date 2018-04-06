/* com_zmp_model - COM-ZMP model
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

#include "holon/corelib/humanoid/com_zmp_model.hpp"

#include <roki/rk_g.h>
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

namespace holon {

using com_zmp_model_formula::computeSqrZeta;
using com_zmp_model_formula::computeZeta;
using com_zmp_model_formula::computeReactForce;
using com_zmp_model_formula::isMassValid;
using com_zmp_model_formula::isComZmpDiffValid;
using com_zmp_model_formula::isReactionForceValid;

ComZmpModel::ComZmpModel()
    : ComZmpModel(RawData::default_com_position, RawData::default_mass) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position)
    : ComZmpModel(t_com_position, RawData::default_mass) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position, double t_mass)
    : ComZmpModel(isMassValid(t_mass) ? make_data<Data>(t_com_position, t_mass)
                                      : make_data<Data>(t_com_position)) {}

ComZmpModel::ComZmpModel(Data t_data)
    : ModelBase(t_data), m_initial_com_position(this->states().com_position) {}

ComZmpModel& ComZmpModel::set_initial_com_position(
    const Vec3D& t_initial_com_position) {
  m_initial_com_position = t_initial_com_position;
  return *this;
}

ComZmpModel& ComZmpModel::reset() {
  states().com_position = m_initial_com_position;
  states().com_velocity.clear();
  Base::reset();
  return *this;
}

ComZmpModel& ComZmpModel::reset(const Vec3D& t_com_position) {
  set_initial_com_position(t_com_position);
  return reset();
}

ComZmpModel& ComZmpModel::setExternalForceCallback(CallbackFunc t_f) {
  system().set_external_force_f(t_f);
  return *this;
}

ComZmpModel& ComZmpModel::setReactionForceCallback(CallbackFunc t_f) {
  system().set_reaction_force_f(t_f);
  return *this;
}

ComZmpModel& ComZmpModel::setZmpPositionCallback(CallbackFunc t_f) {
  system().set_zmp_position_f(t_f);
  return *this;
}

ComZmpModel& ComZmpModel::setComAccelerationCallback(CallbackFunc t_f) {
  system().set_com_acceleration_f(t_f);
  return *this;
}

ComZmpModel& ComZmpModel::setZmpPosition(const Vec3D& t_zmp_position,
                                         optional<double> t_reaction_force_z) {
  double fz = t_reaction_force_z.value_or(mass() * RK_G);
  auto fz_f = [fz](const Vec3D&, const Vec3D&, const double) {
    return Vec3D(0, 0, fz);
  };
  system().set_reaction_force_f(fz_f);
  auto zmp_f = [t_zmp_position](const Vec3D&, const Vec3D&, const double) {
    return t_zmp_position;
  };
  system().set_zmp_position_f(zmp_f);
  return *this;
}

ComZmpModel& ComZmpModel::setReactionForce(const Vec3D& t_reaction_force) {
  system().set_reaction_force_f([t_reaction_force](
      const Vec3D&, const Vec3D&, const double) { return t_reaction_force; });
  return *this;
}

ComZmpModel& ComZmpModel::setExternalForce(const Vec3D& t_external_force) {
  system().set_external_force_f([t_external_force](
      const Vec3D&, const Vec3D&, const double) { return t_external_force; });
  return *this;
}

ComZmpModel& ComZmpModel::removeZmpPosition() {
  system().set_reaction_force_f(system().getDefaultReactForceFunc());
  system().set_zmp_position_f(system().getDefaultZmpPosFunc());
  return *this;
}

ComZmpModel& ComZmpModel::removeReactionForce() {
  system().set_reaction_force_f(system().getDefaultReactForceFunc());
  return *this;
}

ComZmpModel& ComZmpModel::removeExternalForce() {
  system().set_external_force_f(system().getDefaultExtForceFunc());
  return *this;
}

bool ComZmpModel::isUpdatable(const Vec3D& p, const Vec3D& v) {
  if (!isMassValid(mass())) return false;
  if (system().is_set_zmp_position()) {
    auto pz = system().zmp_position(p, v, time());
    if (!isComZmpDiffValid(p, pz)) return false;
  } else {
    auto f = system().reaction_force(p, v, time());
    if (!isReactionForceValid(f)) return false;
  }
  return true;
}

void ComZmpModel::updateData(const Vec3D& p, const Vec3D& v) {
  std::array<Vec3D, 2> state{{p, v}};
  state = solver().update(system(), state, time(), time_step());
  states().com_position = state[0];
  states().com_velocity = state[1];
  states().com_acceleration = system().com_acceleration(p, v, time());
  if (system().is_set_zmp_position()) {
    states().zmp_position = system().zmp_position(p, v, time());
    auto fz = system().reaction_force(p, v, time()).z();
    states().reaction_force = computeReactForce(p, states().zmp_position, fz);
  } else {
    states().reaction_force = system().reaction_force(p, v, time());
  }
  states().external_force = system().external_force(p, v, time());
  states().total_force = states().reaction_force + states().external_force;
}

bool ComZmpModel::update() {
  auto p = states().com_position;
  auto v = states().com_velocity;
  if (!isUpdatable(p, v)) return false;
  updateData(p, v);
  if (!Base::update()) return false;
  return true;
}

bool ComZmpModel::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
