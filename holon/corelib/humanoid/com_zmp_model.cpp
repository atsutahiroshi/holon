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
}  //

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
    : m_data_ptr(t_data_ptr) {
  std::cout << "count = " << m_data_ptr.use_count() << "\n";
}

ComZmpModelSystem::self_ref ComZmpModelSystem::set_data_ptr(
    DataPtr t_data_ptr) {
  m_data_ptr = t_data_ptr;
  return *this;
}

ComZmpModel::ComZmpModel()
    : m_data_ptr(createComZmpModelData()),
      m_initial_com_position(m_data_ptr->com_position),
      m_time_step(default_time_step) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position)
    : m_data_ptr(createComZmpModelData(t_com_position)),
      m_initial_com_position(m_data_ptr->com_position),
      m_time_step(default_time_step) {}

ComZmpModel::ComZmpModel(const Vec3D& t_com_position, double t_mass)
    : m_data_ptr(isMassValid(t_mass)
                     ? createComZmpModelData(t_com_position, t_mass)
                     : createComZmpModelData(t_com_position)),
      m_initial_com_position(m_data_ptr->com_position),
      m_time_step(default_time_step) {}

ComZmpModel::ComZmpModel(DataPtr t_data)
    : m_data_ptr(t_data),
      m_initial_com_position(m_data_ptr->com_position),
      m_time_step(default_time_step) {}

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
  m_external_force_f = t_f;
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setReactionForceCallback(CallbackFunc t_f) {
  m_reaction_force_f = t_f;
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setZmpPositionCallback(CallbackFunc t_f) {
  m_zmp_position_f = t_f;
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setComAccelerationCallback(
    CallbackFunc t_f) {
  m_com_acceleration_f = t_f;
  return *this;
}

ComZmpModel::self_ref ComZmpModel::setZmpPos(
    const Vec3D& t_zmp_position, optional<double> t_reaction_force_z) {
  double fz = t_reaction_force_z.value_or(mass() * RK_G);
  auto fz_f = [fz](double, const Vec3D&, const Vec3D&) {
    return Vec3D(0, 0, fz);
  };
  setReactionForceCallback(fz_f);
  auto zmp_f = [t_zmp_position](double, const Vec3D&, const Vec3D&) {
    return t_zmp_position;
  };
  return setZmpPositionCallback(zmp_f);
}

ComZmpModel::self_ref ComZmpModel::setExternalForce(
    const Vec3D& t_external_force) {
  auto ef_f = [t_external_force](double, const Vec3D&, const Vec3D&) {
    return t_external_force;
  };
  return setExternalForceCallback(ef_f);
}

ComZmpModel::CallbackFunc ComZmpModel::getDefaultReactionForceUpdater() {
  return [this](double, const Vec3D&, const Vec3D&) {
    return Vec3D(0, 0, mass() * RK_G);
  };
}

ComZmpModel::CallbackFunc ComZmpModel::getDefaultExternalForceUpdater() {
  return [this](double, const Vec3D&, const Vec3D&) { return kVec3DZero; };
}

ComZmpModel::CallbackFunc ComZmpModel::getComAccUpdaterViaZmp() {
  return [this](double t, const Vec3D& p, const Vec3D& v) {
    auto f = m_reaction_force_f(t, p, v);
    auto ef = m_external_force_f(t, p, v);
    auto pz = m_zmp_position_f(t, p, v);
    auto ddp = computeComAcc(p, pz, f, mass(), ef);
    return ddp;
  };
}

ComZmpModel::CallbackFunc ComZmpModel::getComAccUpdaterViaForce() {
  return [this](double t, const Vec3D& p, const Vec3D& v) {
    auto f = m_reaction_force_f(t, p, v);
    auto ef = m_external_force_f(t, p, v);
    auto ddp = computeComAcc(f, mass(), ef);
    return ddp;
  };
}

ComZmpModel::CallbackFunc ComZmpModel::getComAccUpdater() {
  if (!m_reaction_force_f)
    m_reaction_force_f = getDefaultReactionForceUpdater();
  if (!m_external_force_f)
    m_external_force_f = getDefaultExternalForceUpdater();
  if (m_zmp_position_f) {
    return [this](double t, const Vec3D& p, const Vec3D& v) {
      auto f = m_reaction_force_f(t, p, v);
      auto ef = m_external_force_f(t, p, v);
      auto pz = m_zmp_position_f(t, p, v);
      return computeComAcc(p, pz, f, mass(), ef);
    };
  } else {
    return [this](double t, const Vec3D& p, const Vec3D& v) {
      auto f = m_reaction_force_f(t, p, v);
      auto ef = m_external_force_f(t, p, v);
      return computeComAcc(f, mass(), ef);
    };
  }
}

std::pair<Vec3D, Vec3D> ComZmpModel::rk4_cat(std::pair<Vec3D, Vec3D> x,
                                             double dt,
                                             std::pair<Vec3D, Vec3D> dx) {
  return std::make_pair(x.first + dt * dx.first, x.second + dt * dx.second);
}

std::pair<Vec3D, Vec3D> ComZmpModel::rk4_f(double t,
                                           std::pair<Vec3D, Vec3D> x) {
  return std::make_pair(x.second, m_com_acceleration_f(t, x.first, x.second));
}

std::pair<Vec3D, Vec3D> ComZmpModel::updateRk4(double t,
                                               std::pair<Vec3D, Vec3D> x,
                                               double dt) {
  std::pair<Vec3D, Vec3D> k1, k2, k3, k4, xm;
  double dt1, dt2, dt3;

  dt1 = dt * 0.5;
  dt2 = dt / 6;
  dt3 = dt2 * 2;

  k1 = rk4_f(t, x);
  // std::cout << "x  = " << x.first << ", " << x.second << "\n";
  // std::cerr << "k1 = " << k1.first << ", " << k1.second << "\n";
  xm = rk4_cat(x, dt1, k1);
  k2 = rk4_f(t + dt1, xm);
  // std::cout << "xm = " << xm.first << ", " << xm.second << "\n";
  // std::cerr << "k2 = " << k2.first << ", " << k2.second << "\n";
  xm = rk4_cat(x, dt1, k2);
  k3 = rk4_f(t + dt1, xm);
  // std::cout << "xm = " << xm.first << ", " << xm.second << "\n";
  // std::cerr << "k3 = " << k3.first << ", " << k3.second << "\n";
  xm = rk4_cat(x, dt, k3);
  k4 = rk4_f(t + dt, xm);
  // std::cout << "xm = " << xm.first << ", " << xm.second << "\n";
  // std::cerr << "k4 = " << k4.first << ", " << k4.second << "\n";

  xm = x;
  xm = rk4_cat(xm, dt2, k1);
  xm = rk4_cat(xm, dt3, k2);
  xm = rk4_cat(xm, dt3, k3);
  xm = rk4_cat(xm, dt2, k4);
  // std::cout << "xm = " << xm.first << ", " << xm.second << "\n";
  return xm;
}

std::pair<Vec3D, Vec3D> ComZmpModel::updateEuler(double t,
                                                 std::pair<Vec3D, Vec3D> x,
                                                 double dt) {
  return rk4_cat(x, dt, rk4_f(t, x));
}

#if 1
bool ComZmpModel::update() {
  if (!m_com_acceleration_f) {
    m_com_acceleration_f = getComAccUpdater();
  }
  auto p = data().com_position;
  auto v = data().com_velocity;
  m_data_ptr->com_acceleration = m_com_acceleration_f(0, p, v);
  m_data_ptr->reaction_force = m_reaction_force_f(0, p, v);
  m_data_ptr->external_force = m_external_force_f(0, p, v);
  m_data_ptr->total_force = data().reaction_force + data().external_force;
  if (m_zmp_position_f) {
    m_data_ptr->zmp_position = m_zmp_position_f(0, p, v);
    double sqr_zeta = computeSqrZeta(p, data().zmp_position,
                                     data().reaction_force, data().mass);
    if (zIsTiny(sqr_zeta)) return false;
  }

  auto ret = updateRk4(0, std::make_pair(p, v), time_step());
  // auto ret = updateEuler(0, std::make_pair(p, v), time_step());
  m_data_ptr->com_position = ret.first;
  m_data_ptr->com_velocity = ret.second;
  m_data_ptr->com_acceleration = m_com_acceleration_f(0, p, v);
  if (m_zmp_position_f) {
    m_data_ptr->reaction_force =
        computeReactForce(p, data().zmp_position, data().reaction_force.z());
  } else {
    m_data_ptr->reaction_force = m_reaction_force_f(0, p, v);
  }
  m_data_ptr->total_force = data().reaction_force + data().external_force;
  m_data_ptr->external_force = kVec3DZero;
  return true;
}
#else
bool ComZmpModel::update() {
  // check if the value of zeta will be valid when computing acceleration
  double sqr_zeta = computeSqrZeta(data().com_position, data().zmp_position,
                                   data().reaction_force, data().mass);
  if (zIsTiny(sqr_zeta)) return false;

  // compute acceleration from reaction force
  Vec3D acc =
      computeComAcc(data().reaction_force, mass(), data().external_force);

  // integrate COM position / velocity by one time step
  Vec3D pos = data().com_position + time_step() * data().com_velocity;
  Vec3D vel = data().com_velocity + time_step() * acc;

  // update stetes
  m_data_ptr->com_position = pos;
  m_data_ptr->com_velocity = vel;
  m_data_ptr->com_acceleration = acc;
  m_data_ptr->total_force = data().reaction_force + data().external_force;

  // remove external force for next step
  clear_external_force();
  return true;
}
#endif

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
