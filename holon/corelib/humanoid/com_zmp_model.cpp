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

ComZmpModelData::ComZmpModelData()
    : m_mass(default_mass),
      m_com_position(default_com_position),
      m_com_velocity({{0, 0, 0}}),
      m_com_acceleration({{0, 0, 0}}),
      m_zmp_position({{0, 0, 0}}) {}

ComZmpModelData::ComZmpModelData(double t_mass)
    : m_mass(default_mass),
      m_com_position(default_com_position),
      m_com_velocity({{0, 0, 0}}),
      m_com_acceleration({{0, 0, 0}}),
      m_zmp_position({{0, 0, 0}}) {
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
    const zVec3D& t_com_position) {
  zVec3DCopy(&t_com_position, &m_com_position);
  return *this;
}

ComZmpModelData& ComZmpModelData::set_com_velocity(
    const zVec3D& t_com_velocity) {
  zVec3DCopy(&t_com_velocity, &m_com_velocity);
  return *this;
}

ComZmpModelData& ComZmpModelData::set_com_acceleration(
    const zVec3D& t_com_acceleration) {
  zVec3DCopy(&t_com_acceleration, &m_com_acceleration);
  return *this;
}

ComZmpModelData& ComZmpModelData::set_zmp_position(
    const zVec3D& t_zmp_position) {
  zVec3DCopy(&t_zmp_position, &m_zmp_position);
  return *this;
}

ComZmpModelData& ComZmpModelData::reset(const zVec3D& t_com_position) {
  set_com_position(t_com_position);
  set_com_velocity(zVec3D({{0, 0, 0}}));
  return *this;
}

ComZmpModel::ComZmpModel() : m_data(), m_time_step(default_time_step) {}

ComZmpModel::ComZmpModel(double t_mass)
    : m_data(t_mass), m_time_step(default_time_step) {}

ComZmpModel& ComZmpModel::set_mass(double t_mass) {
  m_data.set_mass(t_mass);
  return *this;
}

ComZmpModel& ComZmpModel::set_com_position(const zVec3D& t_com_position) {
  m_data.set_com_position(t_com_position);
  return *this;
}

ComZmpModel& ComZmpModel::set_com_velocity(const zVec3D& t_com_velocity) {
  m_data.set_com_velocity(t_com_velocity);
  return *this;
}

ComZmpModel& ComZmpModel::set_com_acceleration(
    const zVec3D& t_com_acceleration) {
  m_data.set_com_acceleration(t_com_acceleration);
  return *this;
}

ComZmpModel& ComZmpModel::set_zmp_position(const zVec3D& t_zmp_position) {
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

ComZmpModel& ComZmpModel::reset(const zVec3D& t_com_position) {
  m_data.reset(t_com_position);
  return *this;
}

double ComZmpModel::computeZetaSqr(const zVec3D& t_com_position) const {
  if (zIsTiny(zVec3DElem(&t_com_position, zZ)) ||
      zVec3DElem(&t_com_position, zZ) < 0) {
    ZRUNERROR("The COM height must be positive. (given: %f)",
              zVec3DElem(&t_com_position, zZ));
    return 0.0;
  }
  return RK_G / zVec3DElem(&t_com_position, zZ);
}

double ComZmpModel::computeZeta(const zVec3D& t_com_position) const {
  return sqrt(computeZetaSqr(t_com_position));
}

zVec3D ComZmpModel::computeAcceleration(const zVec3D& t_com_position,
                                        const zVec3D& t_zmp_position) const {
  zVec3D g = {{0, 0, RK_G}};
  zVec3D com_acceleration;
  // TODO(*): remove const_cast when own math library is implemented
  zVec3DSub(const_cast<zVec3D*>(&t_com_position),
            const_cast<zVec3D*>(&t_zmp_position), &com_acceleration);
  zVec3DMulDRC(&com_acceleration, computeZetaSqr(t_com_position));
  zVec3DSubDRC(&com_acceleration, &g);
  return com_acceleration;
}

bool ComZmpModel::update() {
  if (zIsTiny(computeZetaSqr(com_position()))) return false;
  zVec3D pos = com_position();
  zVec3D vel = com_velocity();
  zVec3D acc = computeAcceleration(com_position(), zmp_position());
  zVec3DCatDRC(&pos, time_step(), &vel);
  zVec3DCatDRC(&vel, time_step(), &acc);
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
