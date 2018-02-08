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

namespace holon {

ComZmpModel::ComZmpModel()
    : m_mass(default_mass), m_step_time(default_step_time) {
  initializeStates();
}
ComZmpModel::ComZmpModel(double t_mass)
    : m_mass(t_mass), m_step_time(default_step_time) {
  initializeStates();
}

ComZmpModel::~ComZmpModel() = default;

void ComZmpModel::initializeStates() {
  zVec3DCreate(&m_com_position, 0, 0, default_com_height);
  zVec3DClear(&m_com_velocity);
  zVec3DClear(&m_com_acceleration);
  zVec3DClear(&m_zmp_position);
}

ComZmpModel& ComZmpModel::set_mass(double t_mass) {
  if (zIsTiny(t_mass) || t_mass < 0) {
    ZRUNERROR("mass must be positive value (given: %f)", t_mass);
    m_mass = default_mass;
  } else {
    m_mass = t_mass;
  }
  return *this;
}

ComZmpModel& ComZmpModel::set_com_position(const zVec3D* t_com_position) {
  zVec3DCopy(t_com_position, &m_com_position);
  return *this;
}

ComZmpModel& ComZmpModel::set_com_velocity(const zVec3D* t_com_velocity) {
  zVec3DCopy(t_com_velocity, &m_com_velocity);
  return *this;
}

ComZmpModel& ComZmpModel::set_zmp_position(const zVec3D* t_zmp_position) {
  zVec3DCopy(t_zmp_position, &m_zmp_position);
  return *this;
}

ComZmpModel& ComZmpModel::set_step_time(double t_step_time) {
  if (zIsTiny(t_step_time) || t_step_time < 0) {
    ZRUNERROR("step time must be positive value (given: %f)", t_step_time);
    m_step_time = default_step_time;
  } else {
    m_step_time = t_step_time;
  }
  return *this;
}

double ComZmpModel::computeZetaSqr(const zVec3D* t_com_position) const {
  if (zIsTiny(zVec3DElem(t_com_position, zZ)) ||
      zVec3DElem(t_com_position, zZ) < 0) {
    ZRUNERROR("The COM height must be positive. (given: %f)",
              zVec3DElem(t_com_position, zZ));
    return 0.0;
  }
  return RK_G / zVec3DElem(t_com_position, zZ);
}

double ComZmpModel::computeZeta(const zVec3D* t_com_position) const {
  return sqrt(computeZetaSqr(t_com_position));
}

zVec3D* ComZmpModel::computeAcceleration(const zVec3D* t_com_position,
                                         const zVec3D* t_zmp_position,
                                         zVec3D* t_com_acceleration) const {
  zVec3D g = {{0, 0, RK_G}};
  // TODO(*): remove const_cast when own math library is implemented
  zVec3DSub(const_cast<zVec3D*>(t_com_position),
            const_cast<zVec3D*>(t_zmp_position), t_com_acceleration);
  zVec3DMulDRC(t_com_acceleration, computeZetaSqr(t_com_position));
  zVec3DSubDRC(t_com_acceleration, &g);
  return t_com_acceleration;
}

bool ComZmpModel::update() {
  computeAcceleration(&m_com_position, &m_zmp_position, &m_com_acceleration);
  return true;
}

bool ComZmpModel::update(double t_step_time) {
  set_step_time(t_step_time);
  return update();
}

}  // namespace holon
