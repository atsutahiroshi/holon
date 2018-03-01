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
#include <memory>
#include <utility>

namespace holon {

const double ComZmpModelData::default_mass = 1.0;
const Vec3D ComZmpModelData::default_com_position = {0.0, 0.0, 1.0};

namespace {
const Vec3D kG = {0, 0, RK_G};
}  // namespace

using ComZmpModelFormula::computeSqrZeta;
using ComZmpModelFormula::computeZeta;
using ComZmpModelFormula::computeReactForce;
using ComZmpModelFormula::computeComAcc;
using ComZmpModelFormula::isMassValid;
using ComZmpModelFormula::isComZmpDiffValid;
using ComZmpModelFormula::isReactionForceValid;
using ComZmpModelFormula::isComAccelerationValid;

ComZmpModelData::ComZmpModelData(const Vec3D& t_com_position, double t_mass)
    : mass(t_mass),
      nu(kVec3DZ),
      com_position(t_com_position),
      com_velocity(kVec3DZero),
      com_acceleration(kVec3DZero),
      zmp_position(kVec3DZero),
      reaction_force(0, 0, t_mass * RK_G),
      external_force(kVec3DZero),
      total_force(reaction_force) {}

std::shared_ptr<ComZmpModelData> createComZmpModelData() {
  return std::make_shared<ComZmpModelData>();
}

std::shared_ptr<ComZmpModelData> createComZmpModelData(
    const Vec3D& t_com_position) {
  return std::make_shared<ComZmpModelData>(t_com_position);
}

std::shared_ptr<ComZmpModelData> createComZmpModelData(
    const Vec3D& t_com_position, double t_mass) {
  return std::make_shared<ComZmpModelData>(t_com_position, t_mass);
}

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

ComZmpModelSystem::Function ComZmpModelSystem::getDefaultComAccFunc() {
  return
      [this](const Vec3D&, const Vec3D&, const double) { return kVec3DZero; };
}

ComZmpModelSystem::Function ComZmpModelSystem::getComAccFuncWithReactForce() {
  return [this](const Vec3D& p, const Vec3D& v, const double t) {
    return computeComAcc(m_reaction_force_f(p, v, t), m_data_ptr->mass,
                         m_external_force_f(p, v, t));
  };
}
ComZmpModelSystem::Function ComZmpModelSystem::getComAccFuncWithZmpPos() {
  return [this](const Vec3D& p, const Vec3D& v, const double t) {
    return computeComAcc(p, m_zmp_position_f(p, v, t),
                         m_reaction_force_f(p, v, t), m_data_ptr->mass,
                         m_external_force_f(p, v, t));
  };
}

ComZmpModelSystem::Function ComZmpModelSystem::getDefaultReactForceFunc() {
  return [this](const Vec3D&, const Vec3D&, const double) {
    return Vec3D(0, 0, m_data_ptr->mass * RK_G);
  };
}

ComZmpModelSystem::Function ComZmpModelSystem::getDefaultExtForceFunc() {
  return
      [this](const Vec3D&, const Vec3D&, const double) { return kVec3DZero; };
}

ComZmpModelSystem::Function ComZmpModelSystem::getDefaultZmpPosFunc() {
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
  cat(x, dt, dx, x);
  return x;
}

}  // namespace integrator

ComZmpModel::ComZmpModel()
    : m_data_ptr(createComZmpModelData()),
      m_initial_com_position(m_data_ptr->com_position),
      m_time_step(default_time_step),
      m_system(m_data_ptr) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position)
    : m_data_ptr(createComZmpModelData(t_com_position)),
      m_initial_com_position(m_data_ptr->com_position),
      m_time_step(default_time_step),
      m_system(m_data_ptr) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position, double t_mass)
    : m_data_ptr(isMassValid(t_mass)
                     ? createComZmpModelData(t_com_position, t_mass)
                     : createComZmpModelData(t_com_position)),
      m_initial_com_position(m_data_ptr->com_position),
      m_time_step(default_time_step),
      m_system(m_data_ptr) {}

ComZmpModel::ComZmpModel(DataPtr t_data)
    : m_data_ptr(t_data),
      m_initial_com_position(m_data_ptr->com_position),
      m_time_step(default_time_step),
      m_system(m_data_ptr) {}

ComZmpModel::self_ref ComZmpModel::set_data_ptr(DataPtr t_data_ptr) {
  m_data_ptr = t_data_ptr;
  return *this;
}

ComZmpModel::self_ref ComZmpModel::set_external_force(
    const Vec3D& t_external_force) {
  m_data_ptr->external_force = t_external_force;
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
  return *this;
}

void ComZmpModel::copy_data(const ComZmpModel& t_model) {
  copy_data(t_model.data());
}

void ComZmpModel::copy_data(const Data& t_data) { *m_data_ptr = t_data; }

void ComZmpModel::inputZmpPos(const Vec3D& t_zmp_position,
                              optional<double> t_reaction_force_z) {
  double fz = t_reaction_force_z.value_or(mass() * RK_G);
  m_data_ptr->zmp_position = t_zmp_position;
  inputReactForce(
      computeReactForce(data().com_position, data().zmp_position, fz));
}

void ComZmpModel::inputReactForce(const Vec3D& t_reaction_force) {
  m_data_ptr->reaction_force = t_reaction_force;
}

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

ComZmpModel::self_ref ComZmpModel::setZmpPos(
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

ComZmpModel::self_ref ComZmpModel::setExternalForce(
    const Vec3D& t_external_force) {
  auto ef_f = [t_external_force](const Vec3D&, const Vec3D&, const double) {
    return t_external_force;
  };
  m_system.set_external_force_f(ef_f);
  return *this;
}

bool ComZmpModel::update() {
  auto p = data().com_position;
  auto v = data().com_velocity;
  if (m_system.isZmpPositionSet()) {
    auto zmp = m_system.zmp_position(p, v, 0.0);
    auto f = m_system.reaction_force(p, v, 0.0);
    auto zeta2 = computeSqrZeta(p, zmp, f, data().mass);
    if (zIsTiny(zeta2)) return false;
  }
  std::array<Vec3D, 2> state{{p, v}};
  auto ret = integrator::update(m_system, state, 0.0, time_step());
  m_data_ptr->com_position = ret[0];
  m_data_ptr->com_velocity = ret[1];
  m_data_ptr->com_acceleration = m_system.com_acceleration(p, v, 0.0);
  if (m_system.isZmpPositionSet()) {
    m_data_ptr->zmp_position = m_system.zmp_position(p, v, 0.0);
    auto fz = m_system.reaction_force(p, v, 0.0).z();
    m_data_ptr->reaction_force = computeReactForce(p, data().zmp_position, fz);
  } else {
    m_data_ptr->reaction_force = m_system.reaction_force(p, v, 0.0);
  }
  m_data_ptr->external_force = m_system.external_force(p, v, 0.0);
  m_data_ptr->total_force = data().reaction_force + data().external_force;
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

namespace ComZmpModelFormula {

double computeSqrZeta(double t_com_position_z, double t_zmp_position_z,
                      double t_com_acceleration_z) {
  if (!isComZmpDiffValid(t_com_position_z, t_zmp_position_z) ||
      !isComAccelerationValid(t_com_acceleration_z)) {
    return 0.0;
  }
  double numer = t_com_acceleration_z + RK_G;
  double denom = t_com_position_z - t_zmp_position_z;
  return numer / denom;
}

double computeSqrZeta(double t_com_position_z, double t_zmp_position_z,
                      double t_reaction_force_z, double t_mass) {
  if (!isMassValid(t_mass) ||
      !isComZmpDiffValid(t_com_position_z, t_zmp_position_z) ||
      !isReactionForceValid(t_reaction_force_z)) {
    return 0.0;
  }
  double denom = (t_com_position_z - t_zmp_position_z) * t_mass;
  return t_reaction_force_z / denom;
}

double computeSqrZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                      const Vec3D& t_com_acceleration, const Vec3D& t_nu) {
  if (!isComZmpDiffValid(t_com_position, t_zmp_position) ||
      !isComAccelerationValid(t_com_acceleration)) {
    return 0.0;
  }
  double numer = t_nu.dot(t_com_acceleration + kG);
  double denom = t_nu.dot(t_com_position - t_zmp_position);
  return numer / denom;
}

double computeSqrZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                      const Vec3D& t_reaction_force, double t_mass,
                      const Vec3D& t_nu) {
  if (!isMassValid(t_mass) ||
      !isComZmpDiffValid(t_com_position, t_zmp_position) ||
      !isReactionForceValid(t_reaction_force)) {
    return 0.0;
  }
  double numer = t_nu.dot(t_reaction_force);
  double denom = t_nu.dot(t_com_position - t_zmp_position) * t_mass;
  return numer / denom;
}

double computeZeta(double t_com_position_z, double t_zmp_position_z,
                   double t_com_acceleration_z) {
  return sqrt(
      computeSqrZeta(t_com_position_z, t_zmp_position_z, t_com_acceleration_z));
}

double computeZeta(double t_com_position_z, double t_zmp_position_z,
                   double t_reation_force_z, double t_mass) {
  return sqrt(computeSqrZeta(t_com_position_z, t_zmp_position_z,
                             t_reation_force_z, t_mass));
}

double computeZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                   const Vec3D& t_com_acceleration, const Vec3D& t_nu) {
  return sqrt(
      computeSqrZeta(t_com_position, t_zmp_position, t_com_acceleration, t_nu));
}

double computeZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                   const Vec3D& t_reaction_force, double t_mass,
                   const Vec3D& t_nu) {
  return sqrt(computeSqrZeta(t_com_position, t_zmp_position, t_reaction_force,
                             t_mass, t_nu));
}

Vec3D computeReactForce(const Vec3D& t_com_acceleration, double t_mass) {
  return t_mass * (t_com_acceleration + kG);
}

Vec3D computeReactForce(const Vec3D& t_com_position,
                        const Vec3D& t_zmp_position, double t_sqr_zeta,
                        double t_mass) {
  return t_mass * t_sqr_zeta * (t_com_position - t_zmp_position);
}

Vec3D computeReactForce(const Vec3D& t_com_position,
                        const Vec3D& t_zmp_position,
                        const Vec3D& t_com_acceleration, double t_mass,
                        const Vec3D& t_nu) {
  double sqr_zeta =
      computeSqrZeta(t_com_position, t_zmp_position, t_com_acceleration, t_nu);
  return computeReactForce(t_com_position, t_zmp_position, sqr_zeta, t_mass);
}

Vec3D computeReactForce(const Vec3D& t_com_position,
                        const Vec3D& t_zmp_position,
                        double t_reaction_force_z) {
  if (!isComZmpDiffValid(t_com_position, t_zmp_position))
    return Vec3D(0, 0, t_reaction_force_z);
  double m_sqr_zeta =
      t_reaction_force_z / (t_com_position.z() - t_zmp_position.z());
  return m_sqr_zeta * (t_com_position - t_zmp_position);
}

Vec3D computeComAcc(const Vec3D& t_reaction_force, double t_mass,
                    const Vec3D& t_external_force) {
  return (t_reaction_force + t_external_force) / t_mass - kG;
}

Vec3D computeComAcc(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                    double t_sqr_zeta, double t_mass,
                    const Vec3D& t_external_force) {
  return t_sqr_zeta * (t_com_position - t_zmp_position) - kG +
         t_external_force / t_mass;
}
Vec3D computeComAcc(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                    const Vec3D& t_reaction_force, double t_mass,
                    const Vec3D& t_external_force, const Vec3D& t_nu) {
  double sqr_zeta = computeSqrZeta(t_com_position, t_zmp_position,
                                   t_reaction_force, t_mass, t_nu);
  return computeComAcc(t_com_position, t_zmp_position, sqr_zeta, t_mass,
                       t_external_force);
}

bool isMassValid(double t_mass) {
  if (zIsTiny(t_mass) || t_mass < 0.0) {
    ZRUNWARN("The mass must be positive. (given mass = %g)", t_mass);
    return false;
  }
  return true;
}

bool isComZmpDiffValid(double t_com_position_z, double t_zmp_position_z) {
  double diff = t_com_position_z - t_zmp_position_z;
  if (zIsTiny(diff) || diff < 0.0) {
    ZRUNWARN("The COM must be above the terrain. (given z = %g, zz = %g)",
             t_com_position_z, t_zmp_position_z);
    return false;
  }
  return true;
}

bool isComZmpDiffValid(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                       const Vec3D& t_nu) {
  double com_side = t_nu.dot(t_com_position - t_zmp_position);
  if (zIsTiny(com_side) || com_side < 0.0) {
    ZRUNWARN(
        "The must be above the terrain. "
        "(COM: %s, ZMP: %s, nu: %s)",
        t_com_position.str().c_str(), t_zmp_position.str().c_str(),
        t_nu.str().c_str());
    return false;
  }
  return true;
}

bool isReactionForceValid(double t_reaction_force_z) {
  if (t_reaction_force_z < 0.0) {
    ZRUNWARN("The reaction force must be positive. (given %g)",
             t_reaction_force_z);
    return false;
  }
  return true;
}

bool isReactionForceValid(const Vec3D& t_reaction_force, const Vec3D& t_nu) {
  double react_force = t_nu.dot(t_reaction_force);
  if (react_force < 0.0) {
    ZRUNWARN(
        "The reaction force must be positive. "
        "(force: %s, nu: %s)",
        t_reaction_force.str().c_str(), t_nu.str().c_str());
    return false;
  }
  return true;
}

bool isComAccelerationValid(double t_com_acceleration_z) {
  if ((t_com_acceleration_z + RK_G) < 0.0) {
    ZRUNWARN("The COM acceleration must be greater than -G. (given %g)",
             t_com_acceleration_z);
    return false;
  }
  return true;
}

bool isComAccelerationValid(const Vec3D& t_com_acceleration,
                            const Vec3D& t_nu) {
  double acc = t_nu.dot(t_com_acceleration + kG);
  if (acc < 0.0) {
    ZRUNWARN("Cannot produce pulling force (COM acc: %s, nu: %s)",
             t_com_acceleration.str().c_str(), t_nu.str().c_str());
    return false;
  }
  return true;
}

}  // namespace ComZmpModelFormula

}  // namespace holon
