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

#ifndef HOLON_HUMANOID_BIPED_MODEL_HPP_
#define HOLON_HUMANOID_BIPED_MODEL_HPP_

#include <array>
#include <functional>
#include <memory>
#include "holon/corelib/control/model_base.hpp"
#include "holon/corelib/control/point_mass_model.hpp"
#include "holon/corelib/control/point_mass_model/point_mass_model_data.hpp"
#include "holon/corelib/control/system_base.hpp"
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/humanoid/com_zmp_model.hpp"
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_data.hpp"
#include "holon/corelib/math/ode_runge_kutta4.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

class BipedModelData
    : public DataSetBase<ComZmpModelRawData, PointMassModelRawData<Vec3D>,
                         PointMassModelRawData<Vec3D>> {
  HOLON_DEFINE_DEFAULT_DATA_CTOR(BipedModelData);

  using Self = BipedModelData;
  using TrunkRawDataType = ComZmpModelRawData;
  using FootRawDataType = PointMassModelRawData<Vec3D>;

 public:
  using TrunkDataIndex = index_seq<0>;
  using LeftFootDataIndex = index_seq<1>;
  using RightFootDataIndex = index_seq<2>;
  TrunkDataIndex trunk_data_index;
  LeftFootDataIndex left_foot_data_index;
  RightFootDataIndex right_foot_data_index;

  BipedModelData(const Vec3D& t_com_position, double t_mass,
                 double t_foot_dist);
  BipedModelData(const Vec3D& t_com_position, double t_mass,
                 const Vec3D& t_lf_position, const Vec3D& t_rf_position);

  const TrunkRawDataType& trunk() const { return get<0>(); }
  const FootRawDataType& left_foot() const { return get<1>(); }
  const FootRawDataType& right_foot() const { return get<2>(); }
  TrunkRawDataType& trunk() { return get<0>(); }
  FootRawDataType& left_foot() { return get<1>(); }
  FootRawDataType& right_foot() { return get<2>(); }

  Self& set_foot_dist(double t_foot_dist);
};

class BipedModel : public ModelBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>,
                                    BipedModelData> {
  using Self = BipedModel;
  using Solver = RungeKutta4<std::array<Vec3D, 2>>;
  using Base = ModelBase<Vec3D, Solver, BipedModelData>;

 public:
  using Data = BipedModelData;
  using TrunkModel = ComZmpModel;
  using FootModel = PointMassModel<Vec3D>;
  using TrunkModelData = ComZmpModelData;
  using FootModelData = PointMassModelData<Vec3D>;

 public:
  BipedModel();
  BipedModel(const Vec3D& t_com_position, double t_mass, double t_foot_dist);
  BipedModel(const Vec3D& t_com_position, double t_mass,
             const Vec3D& t_left_foot_position,
             const Vec3D& t_right_foot_position);
  explicit BipedModel(Data t_data);
  virtual ~BipedModel() = default;

  // accessors
  const TrunkModel& trunk() const { return m_trunk; }
  const FootModel& left_foot() const { return m_left_foot; }
  const FootModel& right_foot() const { return m_right_foot; }
  TrunkModel& trunk() { return m_trunk; }
  FootModel& left_foot() { return m_left_foot; }
  FootModel& right_foot() { return m_right_foot; }

  // reset
  virtual Self& reset() override;
  virtual Self& reset(const Vec3D& t_com_position, double t_foot_dist);
  virtual Self& reset(const Vec3D& t_com_position,
                      const Vec3D& t_left_foot_position,
                      const Vec3D& t_right_foot_position);

 private:
  TrunkModel m_trunk;
  FootModel m_left_foot;
  FootModel m_right_foot;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_MODEL_HPP_
