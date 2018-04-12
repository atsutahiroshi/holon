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

#ifndef HOLON_HUMANOID_COM_ZMP_MODEL_DATA_HPP_
#define HOLON_HUMANOID_COM_ZMP_MODEL_DATA_HPP_

#include <memory>
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

struct ComZmpModelRawData : RawDataBase {
  static const double default_mass;
  static const Vec3D default_com_position;
  double mass = default_mass;
  Vec3D nu = kVec3DZ;
  Vec3D com_position = kVec3DZ;
  Vec3D com_velocity = kVec3DZero;
  Vec3D com_acceleration = kVec3DZero;
  Vec3D zmp_position = kVec3DZero;
  Vec3D reaction_force = kVec3DZero;
  Vec3D external_force = kVec3DZero;
  Vec3D total_force = kVec3DZero;
  ComZmpModelRawData() = default;
  ComZmpModelRawData(double t_mass, Vec3D t_com_position);
  ComZmpModelRawData(double t_mass, Vec3D t_nu, Vec3D t_com_position,
                     Vec3D t_com_velocity, Vec3D t_com_acceleration,
                     Vec3D t_zmp_position, Vec3D t_reaction_force,
                     Vec3D t_external_force, Vec3D t_total_force);
};

class ComZmpModelData : public DataSetBase<ComZmpModelRawData> {
  HOLON_DEFINE_DEFAULT_DATA_CTOR(ComZmpModelData);
  using RawData = ComZmpModelRawData;

 public:
  ComZmpModelData(const Vec3D& t_com_position = RawData::default_com_position,
                  double t_mass = RawData::default_mass);
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_DATA_HPP_
