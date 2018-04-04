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

struct ComZmpModelRawData {
  double mass;
  Vec3D nu;
  Vec3D com_position;
  Vec3D com_velocity;
  Vec3D com_acceleration;
  Vec3D zmp_position;
  Vec3D reaction_force;
  Vec3D external_force;
  Vec3D total_force;
};

class ComZmpModelData : public DataSetBase<ComZmpModelRawData> {
  using Self = ComZmpModelData;
  using RawData = ComZmpModelRawData;
  using Base = DataSetBase<RawData>;

 public:
  static const double default_mass;
  static const Vec3D default_com_position;

 public:
  ComZmpModelData(const RawData& t_raw_data);
  ComZmpModelData(std::shared_ptr<RawData> t_raw_data_p);
  ComZmpModelData(const Vec3D& t_com_position = default_com_position,
                  double t_mass = default_mass);
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_DATA_HPP_
