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

ComZmpModelData::ComZmpModelData()
    : mass(default_mass),
      nu(kVec3DZ),
      com_position(default_com_position),
      com_velocity(kVec3DZero),
      com_acceleration(kVec3DZero),
      zmp_position(kVec3DZero),
      reaction_force(0, 0, default_mass * RK_G),
      external_force(kVec3DZero) {}

ComZmpModelData::ComZmpModelData(double t_mass)
    : mass(t_mass),
      nu(kVec3DZ),
      com_position(default_com_position),
      com_velocity(kVec3DZero),
      com_acceleration(kVec3DZero),
      zmp_position(kVec3DZero),
      reaction_force(0, 0, t_mass * RK_G),
      external_force(kVec3DZero) {}

ComZmpModel::ComZmpModel() : m_time_step(default_time_step) {
  m_data = std::make_shared<ComZmpModelData>();
}

ComZmpModel::ComZmpModel(double t_mass) : m_time_step(default_time_step) {
  if (isMassValid(t_mass))
    m_data = std::make_shared<ComZmpModelData>(t_mass);
  else
    m_data = std::make_shared<ComZmpModelData>();
}

ComZmpModel::ComZmpModel(ComZmpModel::Data t_data)
    : m_data(t_data), m_time_step(default_time_step) {}

ComZmpModel& ComZmpModel::set_time_step(double t_time_step) {
  if (!isTimeStepValid(t_time_step)) {
    m_time_step = default_time_step;
  } else {
    m_time_step = t_time_step;
  }
  return *this;
}

ComZmpModel& ComZmpModel::reset(const Vec3D& t_com_position) {
  m_data->com_position = t_com_position;
  m_data->com_velocity.clear();
  return *this;
}

double ComZmpModel::computeSqrZeta(double t_com_position_z,
                                   double t_zmp_position_z,
                                   double t_com_acceleration_z) const {
  if (!isComZmpDiffValid(t_com_position_z, t_zmp_position_z) ||
      !isComAccelerationValid(t_com_acceleration_z)) {
    return 0.0;
  }
  double numer = t_com_acceleration_z + RK_G;
  double denom = t_com_position_z - t_zmp_position_z;
  return numer / denom;
}

double ComZmpModel::computeSqrZeta(double t_com_position_z,
                                   double t_zmp_position_z,
                                   double t_reaction_force_z,
                                   double t_mass) const {
  if (!isMassValid(t_mass) ||
      !isComZmpDiffValid(t_com_position_z, t_zmp_position_z) ||
      !isReactionForceValid(t_reaction_force_z)) {
    return 0.0;
  }
  double denom = (t_com_position_z - t_zmp_position_z) * t_mass;
  return t_reaction_force_z / denom;
}

double ComZmpModel::computeSqrZeta(const Vec3D& t_com_position,
                                   const Vec3D& t_zmp_position,
                                   const Vec3D& t_com_acceleration,
                                   const Vec3D& t_nu) const {
  if (!isComZmpDiffValid(t_com_position, t_zmp_position) ||
      !isComAccelerationValid(t_com_acceleration)) {
    return 0.0;
  }
  double numer = t_nu.dot(t_com_acceleration + kG);
  double denom = t_nu.dot(t_com_position - t_zmp_position);
  return numer / denom;
}

double ComZmpModel::computeSqrZeta(const Vec3D& t_com_position,
                                   const Vec3D& t_zmp_position,
                                   const Vec3D& t_reaction_force, double t_mass,
                                   const Vec3D& t_nu) const {
  if (!isMassValid(t_mass) ||
      !isComZmpDiffValid(t_com_position, t_zmp_position) ||
      !isReactionForceValid(t_reaction_force)) {
    return 0.0;
  }
  double numer = t_nu.dot(t_reaction_force);
  double denom = t_nu.dot(t_com_position - t_zmp_position) * t_mass;
  return numer / denom;
}

double ComZmpModel::computeZeta(double t_com_position_z,
                                double t_zmp_position_z,
                                double t_com_acceleration_z) const {
  return sqrt(
      computeSqrZeta(t_com_position_z, t_zmp_position_z, t_com_acceleration_z));
}

double ComZmpModel::computeZeta(double t_com_position_z,
                                double t_zmp_position_z,
                                double t_reation_force_z, double t_mass) const {
  return sqrt(computeSqrZeta(t_com_position_z, t_zmp_position_z,
                             t_reation_force_z, t_mass));
}

double ComZmpModel::computeZeta(const Vec3D& t_com_position,
                                const Vec3D& t_zmp_position,
                                const Vec3D& t_com_acceleration,
                                const Vec3D& t_nu) const {
  return sqrt(
      computeSqrZeta(t_com_position, t_zmp_position, t_com_acceleration, t_nu));
}

double ComZmpModel::computeZeta(const Vec3D& t_com_position,
                                const Vec3D& t_zmp_position,
                                const Vec3D& t_reaction_force, double t_mass,
                                const Vec3D& t_nu) const {
  return sqrt(computeSqrZeta(t_com_position, t_zmp_position, t_reaction_force,
                             t_mass, t_nu));
}

double ComZmpModel::computeSqrZeta(const Vec3D& t_com_position) const {
  // double numer = nu().dot(reaction_force());
  // double denom = nu().dot(com_position()) * mass();
  if (t_com_position.z() == 0.0 || t_com_position.z() < 0.0) {
    ZRUNERROR("The COM height must be positive. (given: %g)",
              t_com_position.z());
    return 0.0;
  }
  return RK_G / t_com_position.z();
  // return numer / denom;
}

double ComZmpModel::computeZeta(const Vec3D& t_com_position) const {
  return sqrt(computeSqrZeta(t_com_position));
}

Vec3D ComZmpModel::computeComAcc(const Vec3D& t_com_position,
                                 const Vec3D& t_zmp_position) const {
  double zeta2 = computeSqrZeta(t_com_position);
  Vec3D com_acc;
  // TODO(*): remove const_cast when own math library is implemented
  com_acc = zeta2 * (t_com_position - t_zmp_position) - kG;
  // Vec3DSub(const_cast<Vec3D*>(&t_com_position),
  //          const_cast<Vec3D*>(&t_zmp_position), &com_acc);
  // Vec3DMulDRC(&com_acc, computeSqrZeta(t_com_position));
  // Vec3DSubDRC(&com_acc, &g);
  return com_acc;
}

bool ComZmpModel::update() {
  if (zIsTiny(computeSqrZeta(m_data->com_position))) return false;
  Vec3D acc = computeComAcc(m_data->com_position, m_data->zmp_position);
  Vec3D pos = m_data->com_position + time_step() * m_data->com_velocity;
  Vec3D vel = m_data->com_velocity + time_step() * acc;
  m_data->com_position = pos;
  m_data->com_velocity = vel;
  m_data->com_acceleration = acc;
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

bool ComZmpModel::isMassValid(double t_mass) const {
  if (zIsTiny(t_mass) || t_mass < 0.0) {
    ZRUNWARN("The mass must be positive. (given mass = %g)", t_mass);
    return false;
  }
  return true;
}

bool ComZmpModel::isComZmpDiffValid(double t_com_position_z,
                                    double t_zmp_position_z) const {
  double diff = t_com_position_z - t_zmp_position_z;
  if (zIsTiny(diff) || diff < 0.0) {
    ZRUNWARN("The COM must be above the terrain. (given z = %g, zz = %g)",
             t_com_position_z, t_zmp_position_z);
    return false;
  }
  return true;
}

bool ComZmpModel::isComZmpDiffValid(const Vec3D& t_com_position,
                                    const Vec3D& t_zmp_position,
                                    const Vec3D& t_nu) const {
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

bool ComZmpModel::isReactionForceValid(double t_reaction_force_z) const {
  if (t_reaction_force_z < 0.0) {
    ZRUNWARN("The reaction force must be positive. (given %g)",
             t_reaction_force_z);
    return false;
  }
  return true;
}

bool ComZmpModel::isReactionForceValid(const Vec3D& t_reaction_force,
                                       const Vec3D& t_nu) const {
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

bool ComZmpModel::isComAccelerationValid(double t_com_acceleration_z) const {
  if ((t_com_acceleration_z + RK_G) < 0.0) {
    ZRUNWARN("The COM acceleration must be greater than -G. (given %g)",
             t_com_acceleration_z);
    return false;
  }
  return true;
}

bool ComZmpModel::isComAccelerationValid(const Vec3D& t_com_acceleration,
                                         const Vec3D& t_nu) const {
  double acc = t_nu.dot(t_com_acceleration + kG);
  if (acc < 0.0) {
    ZRUNWARN("Cannot produce pulling force (COM acc: %s, nu: %s)",
             t_com_acceleration.str().c_str(), t_nu.str().c_str());
    return false;
  }
  return true;
}

}  // namespace holon
