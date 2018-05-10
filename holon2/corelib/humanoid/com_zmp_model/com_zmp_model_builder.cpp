/* com_zmp_model_builder - Builder class of COM-ZMP model
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of holon.
 *
 * Holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_builder.hpp"
#include "holon2/corelib/humanoid/com_zmp_model.hpp"
#include "holon2/corelib/humanoid/const_defs.hpp"

namespace holon {

const double ComZmpModelBuilder::default_mass = 1.0;
const double ComZmpModelBuilder::default_vhp = 0.0;
const Vec3d ComZmpModelBuilder::default_com_position = {0.0, 0.0, 1.0};

ComZmpModelBuilder::ComZmpModelBuilder()
    : m_mass(default_mass),
      m_vhp(default_vhp),
      m_com_position(default_com_position),
      m_com_velocity(kVec3dZero) {}

ComZmpModelBuilder& ComZmpModelBuilder::setMass(const double t_mass) {
  m_mass = t_mass;
  return *this;
}

ComZmpModelBuilder& ComZmpModelBuilder::setVirtualHorizontalPlane(
    const double t_vhp) {
  m_vhp = t_vhp;
  return *this;
}

ComZmpModelBuilder& ComZmpModelBuilder::setComPosition(
    const Vec3d& t_com_position) {
  m_com_position = t_com_position;
  return *this;
}

ComZmpModelBuilder& ComZmpModelBuilder::setComVelocity(
    const Vec3d& t_com_velocity) {
  m_com_velocity = t_com_velocity;
  return *this;
}

ComZmpModel ComZmpModelBuilder::build() {
  Data data;
  return this->build(data);
}

ComZmpModel ComZmpModelBuilder::build(Data t_data) {
  Vec3d contact_force = m_mass * kGravAccel * kVec3dZ;
  Vec3d zmp_position = {m_com_position[0], m_com_position[1], m_vhp};
  Params params{
      m_mass,   // mass
      kVec3dZ,  // normal vector to virtual plane
      m_vhp     // virtual horizontal plane
  };
  States states{
      m_com_position,  // COM position
      m_com_velocity,  // COM velocity
      kVec3dZero,      // COM accel.
      zmp_position,    // ZMP position
      contact_force,   // contact force
      kVec3dZero,      // external force
      contact_force    // reaction force
  };
  t_data.copy<0, 1>(params, states);
  return ComZmpModel(t_data);
}

}  // namespace holon
