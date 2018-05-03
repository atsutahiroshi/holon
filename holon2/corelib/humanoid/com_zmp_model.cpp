/* com_zmp_model - COM-ZMP model
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

#include "holon2/corelib/humanoid/com_zmp_model.hpp"

namespace holon {

const double ComZmpModelDataBuilder::default_mass = 1.0;
const double ComZmpModelDataBuilder::default_vhp = 0.0;
const Vec3d ComZmpModelDataBuilder::default_com_position = {0.0, 0.0, 1.0};

ComZmpModelDataBuilder::ComZmpModelDataBuilder()
    : m_mass(default_mass),
      m_vhp(default_vhp),
      m_com_position(default_com_position) {}

ComZmpModelDataBuilder& ComZmpModelDataBuilder::setMass(const double t_mass) {
  m_mass = t_mass;
  return *this;
}

ComZmpModelDataBuilder& ComZmpModelDataBuilder::setVirtualHorizontalPlane(
    const double t_vhp) {
  m_vhp = t_vhp;
  return *this;
}

ComZmpModelDataBuilder& ComZmpModelDataBuilder::setComPosition(
    const Vec3d& t_com_position) {
  m_com_position = t_com_position;
  return *this;
}

ComZmpModelData ComZmpModelDataBuilder::build() {
  Data data;
  return this->build(data);
}

ComZmpModelData ComZmpModelDataBuilder::build(Data t_data) {
  Vec3d reaction_force = m_mass * kGravAccel * kVec3dZ;
  Vec3d zmp_position = {m_com_position[0], m_com_position[1], m_vhp};
  Params params{m_mass, kVec3dZ, m_vhp};
  Inputs inputs{m_com_position, kVec3dZero, zmp_position, reaction_force,
                kVec3dZero};
  Outputs outputs{kVec3dZero, kVec3dZero, kVec3dZero,
                  kVec3dZero, kVec3dZero, kVec3dZero};
  t_data.copy<0, 1, 2>(params, inputs, outputs);
  return t_data;
}

}  // namespace holon
