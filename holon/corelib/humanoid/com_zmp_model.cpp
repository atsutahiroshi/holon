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

using ComZmpModelFormula::computeSqrZeta;
using ComZmpModelFormula::computeZeta;
using ComZmpModelFormula::computeReactForce;
using ComZmpModelFormula::isMassValid;

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
    : m_time(0),
      m_time_step(default_time_step),
      m_data_ptr(createComZmpModelData()),
      m_initial_com_position(m_data_ptr->com_position),
      m_system(m_data_ptr) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position)
    : m_time(0),
      m_time_step(default_time_step),
      m_data_ptr(createComZmpModelData(t_com_position)),
      m_initial_com_position(m_data_ptr->com_position),
      m_system(m_data_ptr) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position, double t_mass)
    : m_time(0),
      m_time_step(default_time_step),
      m_data_ptr(isMassValid(t_mass)
                     ? createComZmpModelData(t_com_position, t_mass)
                     : createComZmpModelData(t_com_position)),
      m_initial_com_position(m_data_ptr->com_position),
      m_system(m_data_ptr) {}

ComZmpModel::ComZmpModel(DataPtr t_data)
    : m_time(0),
      m_time_step(default_time_step),
      m_data_ptr(t_data),
      m_initial_com_position(m_data_ptr->com_position),
      m_system(m_data_ptr) {}

ComZmpModel::self_ref ComZmpModel::set_data_ptr(DataPtr t_data_ptr) {
  m_data_ptr = t_data_ptr;
  return *this;
}

ComZmpModel::self_ref ComZmpModel::set_time_step(double t_time_step) {
  if (!isTimeStepValid(t_time_step)) {
    m_time_step = default_time_step;
  } else {
    m_time_step = t_time_step;
  }
  return *this;
}

ComZmpModel::self_ref ComZmpModel::set_initial_com_position(
    const Vec3D& t_initial_com_position) {
  m_initial_com_position = t_initial_com_position;
  return *this;
}

ComZmpModel::self_ref ComZmpModel::reset(const Vec3D& t_com_position) {
  m_data_ptr->com_position = t_com_position;
  m_data_ptr->com_velocity.clear();
  set_initial_com_position(t_com_position);
  m_time = 0;
  return *this;
}

void ComZmpModel::copy_data(const ComZmpModel& t_model) {
  copy_data(t_model.data());
}

void ComZmpModel::copy_data(const Data& t_data) { *m_data_ptr = t_data; }

ComZmpModel::self_ref ComZmpModel::setExternalForceCallback(CallbackFunc t_f) {
  m_system.set_external_force_f(t_f);
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setReactionForceCallback(CallbackFunc t_f) {
  m_system.set_reaction_force_f(t_f);
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setZmpPositionCallback(CallbackFunc t_f) {
  m_system.set_zmp_position_f(t_f);
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setComAccelerationCallback(
    CallbackFunc t_f) {
  m_system.set_com_acceleration_f(t_f);
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setFixedZmpPosition(
    const Vec3D& t_zmp_position, optional<double> t_reaction_force_z) {
  double fz = t_reaction_force_z.value_or(mass() * RK_G);
  auto fz_f = [fz](const Vec3D&, const Vec3D&, const double) {
    return Vec3D(0, 0, fz);
  };
  m_system.set_reaction_force_f(fz_f);
  auto zmp_f = [t_zmp_position](const Vec3D&, const Vec3D&, const double) {
    return t_zmp_position;
  };
  m_system.set_zmp_position_f(zmp_f);
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setFixedReactionForce(
    const Vec3D& t_reaction_force) {
  m_system.set_reaction_force_f([t_reaction_force](
      const Vec3D&, const Vec3D&, const double) { return t_reaction_force; });
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setFixedExternalForce(
    const Vec3D& t_external_force) {
  m_system.set_external_force_f([t_external_force](
      const Vec3D&, const Vec3D&, const double) { return t_external_force; });
  return *this;
}

bool ComZmpModel::update() {
  auto p = data().com_position;
  auto v = data().com_velocity;
  if (m_system.isZmpPositionSet()) {
    auto zmp = m_system.zmp_position(p, v, time());
    auto f = m_system.reaction_force(p, v, time());
    auto zeta2 = computeSqrZeta(p, zmp, f, data().mass);
    if (zIsTiny(zeta2)) return false;
  }
  std::array<Vec3D, 2> state{{p, v}};
  auto ret = integrator::update(m_system, state, time(), time_step());
  // auto ret = integrator::update_euler(m_system, state, time(), time_step());
  m_data_ptr->com_position = ret[0];
  m_data_ptr->com_velocity = ret[1];
  m_data_ptr->com_acceleration = m_system.com_acceleration(p, v, time());
  if (m_system.isZmpPositionSet()) {
    m_data_ptr->zmp_position = m_system.zmp_position(p, v, time());
    auto fz = m_system.reaction_force(p, v, time()).z();
    m_data_ptr->reaction_force = computeReactForce(p, data().zmp_position, fz);
  } else {
    m_data_ptr->reaction_force = m_system.reaction_force(p, v, time());
  }
  m_data_ptr->external_force = m_system.external_force(p, v, time());
  m_data_ptr->total_force = data().reaction_force + data().external_force;
  m_time += time_step();
  return true;
}

bool ComZmpModel::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

bool ComZmpModel::isTimeStepValid(double t_time_step) const {
  if (zIsTiny(t_time_step) || t_time_step < 0) {
    ZRUNWARN("Time step must be positive (given: %g)", t_time_step);
    return false;
  }
  return true;
}

}  // namespace holon
