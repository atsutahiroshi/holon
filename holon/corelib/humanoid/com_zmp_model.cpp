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
#include <utility>

namespace holon {

const double ComZmpModelData::default_mass = 1.0;
const Vec3D ComZmpModelData::default_com_position = {0.0, 0.0, 1.0};

ComZmpModelData::ComZmpModelData()
    : m_mass(default_mass),
      m_com_position(default_com_position),
      m_com_velocity(0, 0, 0),
      m_com_acceleration(0, 0, 0),
      m_zmp_position(0, 0, 0) {}

ComZmpModelData::ComZmpModelData(double t_mass)
    : m_mass(default_mass),
      m_com_position(default_com_position),
      m_com_velocity(0, 0, 0),
      m_com_acceleration(0, 0, 0),
      m_zmp_position(0, 0, 0) {
  set_mass(t_mass);
}

ComZmpModelData& ComZmpModelData::set_mass(double t_mass) {
  if (zIsTiny(t_mass) || t_mass < 0) {
    ZRUNERROR("mass must be positive value (given: %g)", t_mass);
    m_mass = default_mass;
  } else {
    m_mass = t_mass;
  }
  return *this;
}

ComZmpModelData& ComZmpModelData::set_com_position(
    const Vec3D& t_com_position) {
  m_com_position = t_com_position;
  return *this;
}

ComZmpModelData& ComZmpModelData::set_com_velocity(
    const Vec3D& t_com_velocity) {
  m_com_velocity = t_com_velocity;
  return *this;
}

ComZmpModelData& ComZmpModelData::set_com_acceleration(
    const Vec3D& t_com_acceleration) {
  m_com_acceleration = t_com_acceleration;
  return *this;
}

ComZmpModelData& ComZmpModelData::set_zmp_position(
    const Vec3D& t_zmp_position) {
  m_zmp_position = t_zmp_position;
  return *this;
}

ComZmpModelData& ComZmpModelData::reset(const Vec3D& t_com_position) {
  m_com_position = t_com_position;
  m_com_velocity.clear();
  return *this;
}

ComZmpModel::ComZmpModel() : m_data(), m_time_step(default_time_step) {}

ComZmpModel::ComZmpModel(double t_mass)
    : m_data(t_mass), m_time_step(default_time_step) {}

ComZmpModel& ComZmpModel::set_mass(double t_mass) {
  m_data.set_mass(t_mass);
  return *this;
}

ComZmpModel& ComZmpModel::set_com_position(const Vec3D& t_com_position) {
  m_data.set_com_position(t_com_position);
  return *this;
}

ComZmpModel& ComZmpModel::set_com_velocity(const Vec3D& t_com_velocity) {
  m_data.set_com_velocity(t_com_velocity);
  return *this;
}

ComZmpModel& ComZmpModel::set_com_acceleration(
    const Vec3D& t_com_acceleration) {
  m_data.set_com_acceleration(t_com_acceleration);
  return *this;
}

ComZmpModel& ComZmpModel::set_zmp_position(const Vec3D& t_zmp_position) {
  m_data.set_zmp_position(t_zmp_position);
  return *this;
}

ComZmpModel& ComZmpModel::set_time_step(double t_time_step) {
  if (zIsTiny(t_time_step) || t_time_step < 0) {
    ZRUNERROR("step time must be positive value (given: %f)", t_time_step);
    m_time_step = default_time_step;
  } else {
    m_time_step = t_time_step;
  }
  return *this;
}

ComZmpModel& ComZmpModel::reset(const Vec3D& t_com_position) {
  m_data.reset(t_com_position);
  return *this;
}

double ComZmpModel::computeZetaSqr(const Vec3D& t_com_position) const {
  if (t_com_position.z() == 0.0 || t_com_position.z() < 0.0) {
    ZRUNERROR("The COM height must be positive. (given: %g)",
              t_com_position.z());
    return 0.0;
  }
  return RK_G / t_com_position.z();
  // if (zIsTiny(Vec3DElem(&t_com_position, zZ)) ||
  //     Vec3DElem(&t_com_position, zZ) < 0) {
  //   ZRUNERROR("The COM height must be positive. (given: %f)",
  //             Vec3DElem(&t_com_position, zZ));
  //   return 0.0;
  // }
  // return RK_G / Vec3DElem(&t_com_position, zZ);
}

double ComZmpModel::computeZeta(const Vec3D& t_com_position) const {
  return sqrt(computeZetaSqr(t_com_position));
}

Vec3D ComZmpModel::computeComAcc(const Vec3D& t_com_position,
                                 const Vec3D& t_zmp_position) const {
  Vec3D g = {0, 0, RK_G};
  double zeta2 = computeZetaSqr(t_com_position);
  Vec3D com_acc;
  // TODO(*): remove const_cast when own math library is implemented
  com_acc = zeta2 * (t_com_position - t_zmp_position) - g;
  // Vec3DSub(const_cast<Vec3D*>(&t_com_position),
  //          const_cast<Vec3D*>(&t_zmp_position), &com_acc);
  // Vec3DMulDRC(&com_acc, computeZetaSqr(t_com_position));
  // Vec3DSubDRC(&com_acc, &g);
  return com_acc;
}

bool ComZmpModel::update() {
  if (zIsTiny(computeZetaSqr(com_position()))) return false;
  Vec3D acc = computeComAcc(com_position(), zmp_position());
  Vec3D pos = com_position() + time_step() * com_velocity();
  Vec3D vel = com_velocity() + time_step() * acc;
  m_data.set_com_position(pos);
  m_data.set_com_velocity(vel);
  m_data.set_com_acceleration(acc);
  return true;
}

bool ComZmpModel::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
