/* com_zmp_model_data - Data for COM-ZMP model
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

#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_data.hpp"

#include <roki/rk_g.h>
#include <memory>
#include <utility>

namespace holon {

const double ComZmpModelRawData::default_mass = 1.0;
const Vec3D ComZmpModelRawData::default_com_position = {0.0, 0.0, 1.0};

ComZmpModelRawData::ComZmpModelRawData(double t_mass, Vec3D t_com_position)
    : mass(t_mass),
      com_position(t_com_position),
      reaction_force({0, 0, mass * RK_G}),
      total_force(reaction_force) {}

ComZmpModelRawData::ComZmpModelRawData(
    double t_mass, Vec3D t_nu, Vec3D t_com_position, Vec3D t_com_velocity,
    Vec3D t_com_acceleration, Vec3D t_zmp_position, Vec3D t_reaction_force,
    Vec3D t_external_force, Vec3D t_total_force)
    : mass(t_mass),
      nu(t_nu),
      com_position(t_com_position),
      com_velocity(t_com_velocity),
      com_acceleration(t_com_acceleration),
      zmp_position(t_zmp_position),
      reaction_force(t_reaction_force),
      external_force(t_external_force),
      total_force(t_total_force) {}

ComZmpModelData::ComZmpModelData(const Vec3D& t_com_position, double t_mass)
    : ComZmpModelData(
          alloc_raw_data<ComZmpModelRawData>(t_mass, t_com_position)) {}

}  // namespace holon
