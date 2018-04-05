/* biped_model - Biped robot model
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

#include "holon/corelib/humanoid/biped_model.hpp"

namespace holon {

BipedModelData::BipedModelData(const Vec3D& t_com_position, double t_mass,
                               double t_foot_dist)
    : BipedModelData(alloc_raw_data<TrunkRawDataType>(t_mass, t_com_position),
                     alloc_raw_data<FootRawDataType>(),
                     alloc_raw_data<FootRawDataType>()) {
  left_foot().position =
      Vec3D{trunk().com_position.x(),
            trunk().com_position.y() + 0.5 * t_foot_dist, 0.0};
  right_foot().position =
      Vec3D{trunk().com_position.x(),
            trunk().com_position.y() - 0.5 * t_foot_dist, 0.0};
}

BipedModelData::BipedModelData(const Vec3D& t_com_position, double t_mass,
                               const Vec3D& t_lf_position,
                               const Vec3D& t_rf_position)
    : BipedModelData(alloc_raw_data<TrunkRawDataType>(t_mass, t_com_position),
                     alloc_raw_data<FootRawDataType>(1.0, t_lf_position),
                     alloc_raw_data<FootRawDataType>(1.0, t_rf_position)) {}

}  // namespace holon
