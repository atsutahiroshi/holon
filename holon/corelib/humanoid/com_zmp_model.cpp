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

namespace integrator {

template <typename State, typename Time>
void cat(const State& state, Time dt, const State& deriv, State& state_out) {
  auto x = state.begin();
  auto dx = deriv.begin();
  auto x_out = state_out.begin();
  for (; x_out != state_out.end(); ++x, ++dx, ++x_out) {
    *x_out = *x + dt * *dx;
  }
}

template <typename System, typename State, typename Time>
State update(const System& system, const State& initial_state, Time t,
             Time dt) {
  State k1, k2, k3, k4, x;
  Time dt1, dt2, dt3;

  dt1 = dt * 0.5;
  dt2 = dt / 6;
  dt3 = dt2 * 2;

  system(initial_state, k1, t);
  cat(initial_state, dt1, k1, x);
  system(x, k2, t + dt1);
  cat(initial_state, dt1, k2, x);
  system(x, k3, t + dt1);
  cat(initial_state, dt, k3, x);
  system(x, k4, t + dt);

  x = initial_state;
  cat(x, dt2, k1, x);
  cat(x, dt3, k2, x);
  cat(x, dt3, k3, x);
  cat(x, dt2, k4, x);
  return x;
}

template <typename System, typename State, typename Time>
State update_euler(const System& system, const State& initial_state, Time t,
                   Time dt) {
  State x, dx;
  system(initial_state, dx, t);
  cat(initial_state, dt, dx, x);
  return x;
}

}  // namespace integrator

ComZmpModel::ComZmpModel()
    : ComZmpModel(Data::default_com_position, Data::default_mass) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position)
    : ComZmpModel(t_com_position, Data::default_mass) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position, double t_mass)
    : ComZmpModel(isMassValid(t_mass)
                      ? createComZmpModelData(t_com_position, t_mass)
                      : createComZmpModelData(t_com_position)) {}

ComZmpModel::ComZmpModel(DataPtr t_data_ptr)
    : ModelBase(t_data_ptr),
      m_initial_com_position(this->data().com_position) {}

ComZmpModel& ComZmpModel::set_initial_com_position(
    const Vec3D& t_initial_com_position) {
  m_initial_com_position = t_initial_com_position;
  return *this;
}

ComZmpModel& ComZmpModel::reset() {
  data_ptr()->com_position = m_initial_com_position;
  data_ptr()->com_velocity.clear();
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
  if (!isMassValid(data().mass)) return false;
  if (system().isZmpPositionSet()) {
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
  state = integrator::update(system(), state, time(), time_step());
  // state = integrator::update_euler(system(), state, time(), time_step());
  data_ptr()->com_position = state[0];
  data_ptr()->com_velocity = state[1];
  data_ptr()->com_acceleration = system().com_acceleration(p, v, time());
  if (system().isZmpPositionSet()) {
    data_ptr()->zmp_position = system().zmp_position(p, v, time());
    auto fz = system().reaction_force(p, v, time()).z();
    data_ptr()->reaction_force = computeReactForce(p, data().zmp_position, fz);
  } else {
    data_ptr()->reaction_force = system().reaction_force(p, v, time());
  }
  data_ptr()->external_force = system().external_force(p, v, time());
  data_ptr()->total_force = data().reaction_force + data().external_force;
}

bool ComZmpModel::update() {
  auto p = data().com_position;
  auto v = data().com_velocity;
  if (!isUpdatable(p, v)) return false;
  updateData(p, v);
  return Base::update();
}

bool ComZmpModel::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
