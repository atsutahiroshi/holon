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
  set_foot_dist(t_foot_dist);
}

BipedModelData::BipedModelData(const Vec3D& t_com_position, double t_mass,
                               const Vec3D& t_lf_position,
                               const Vec3D& t_rf_position)
    : BipedModelData(alloc_raw_data<TrunkRawDataType>(t_mass, t_com_position),
                     alloc_raw_data<FootRawDataType>(1.0, t_lf_position),
                     alloc_raw_data<FootRawDataType>(1.0, t_rf_position)) {}

BipedModelData& BipedModelData::set_foot_dist(double t_foot_dist) {
  left_foot().position =
      Vec3D{trunk().com_position.x(),
            trunk().com_position.y() + 0.5 * t_foot_dist, 0.0};
  right_foot().position =
      Vec3D{trunk().com_position.x(),
            trunk().com_position.y() - 0.5 * t_foot_dist, 0.0};
  return *this;
}

BipedModel::BipedModel() : BipedModel(make_data<Data>()) {}

BipedModel::BipedModel(const Vec3D& t_com_position, double t_mass,
                       double t_foot_dist)
    : BipedModel(make_data<Data>(t_com_position, t_mass, t_foot_dist)) {}

BipedModel::BipedModel(const Vec3D& t_com_position, double t_mass,
                       const Vec3D& t_left_foot_position,
                       const Vec3D& t_right_foot_position)
    : BipedModel(make_data<Data>(t_com_position, t_mass, t_left_foot_position,
                                 t_right_foot_position)) {}

BipedModel::BipedModel(Data t_data)
    : ModelBase(t_data),
      m_trunk(data().extract<TrunkModelData>(Data::TrunkDataIndex())),
      m_left_foot(data().extract<FootModelData>(Data::LeftFootDataIndex())),
      m_right_foot(data().extract<FootModelData>(Data::RightFootDataIndex())) {}

BipedModel& BipedModel::reset() {
  return reset(trunk().initial_com_position(), left_foot().initial_position(),
               right_foot().initial_position());
}
BipedModel& BipedModel::reset(const Vec3D& t_com_position, double t_foot_dist) {
  return reset(
      t_com_position,
      {t_com_position.x(), t_com_position.y() + 0.5 * t_foot_dist, 0.0},
      {t_com_position.x(), t_com_position.y() - 0.5 * t_foot_dist, 0.0});
}
BipedModel& BipedModel::reset(const Vec3D& t_com_position,
                              const Vec3D& t_left_foot_position,
                              const Vec3D& t_right_foot_position) {
  m_trunk.reset(t_com_position);
  m_left_foot.reset(t_left_foot_position);
  m_right_foot.reset(t_right_foot_position);
  Base::reset();
  return *this;
}

}  // namespace holon
