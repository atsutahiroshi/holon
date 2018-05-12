/* biped_foot_model_builder - Builder class of biped foot model
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

#include "holon2/corelib/humanoid/biped_foot_model/biped_foot_model_builder.hpp"
#include "holon2/corelib/humanoid/biped_foot_model.hpp"
#include "holon2/corelib/humanoid/const_defs.hpp"

namespace holon {

const double BipedFootModelBuilder::default_mass = 1.0;
const Vec3d BipedFootModelBuilder::default_position = {0.0, 0.0, 1.0};

BipedFootModelBuilder::BipedFootModelBuilder()
    : m_mass(default_mass),
      m_position(default_position),
      m_velocity(kVec3dZero) {}

BipedFootModelBuilder& BipedFootModelBuilder::setMass(const double t_mass) {
  m_mass = t_mass;
  return *this;
}

BipedFootModelBuilder& BipedFootModelBuilder::setPosition(
    const Vec3d& t_position) {
  m_position = t_position;
  return *this;
}

BipedFootModelBuilder& BipedFootModelBuilder::setVelocity(
    const Vec3d& t_velocity) {
  m_velocity = t_velocity;
  return *this;
}

BipedFootModel BipedFootModelBuilder::build() {
  Data data;
  return this->build(data);
}

BipedFootModel BipedFootModelBuilder::build(Data t_data) {
  Vec3d force = m_mass * kGravAccel3d;
  Params params{
      m_mass  // mass
  };
  States states{
      m_position,  // position
      m_velocity,  // velocity
      kVec3dZero,  // acceleration
      force        // force
  };
  t_data.copy<0, 1>(params, states);
  return BipedFootModel(t_data);
}

}  // namespace holon
