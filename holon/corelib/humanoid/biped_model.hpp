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
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/humanoid/biped_foot_model.hpp"
#include "holon/corelib/humanoid/com_zmp_model.hpp"
#include "holon/corelib/math/ode_runge_kutta4.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

class BipedModelData
    : public DataSetBase<ComZmpModelRawData, PointMassModelRawData<Vec3D>,
                         PointMassModelRawData<Vec3D>> {
  HOLON_DEFINE_DEFAULT_DATA_CTOR(BipedModelData);

  using Self = BipedModelData;
  using TrunkRawData = ComZmpModelRawData;
  using FootRawData = PointMassModelRawData<Vec3D>;
  using TrunkModelData = ComZmpModelData;
  using FootModelData = BipedFootModelData;

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

  const TrunkRawData& trunk() const { return get<0>(); }
  const FootRawData& left_foot() const { return get<1>(); }
  const FootRawData& right_foot() const { return get<2>(); }
  TrunkRawData& trunk() { return get<0>(); }
  FootRawData& left_foot() { return get<1>(); }
  FootRawData& right_foot() { return get<2>(); }

  Self& set_foot_dist(double t_foot_dist);
  TrunkModelData extract_trunk_model_data() const {
    return extract<TrunkModelData>(TrunkDataIndex());
  }
  FootModelData extract_left_foot_model_data() const {
    return extract<FootModelData>(LeftFootDataIndex());
  }
  FootModelData extract_right_foot_model_data() const {
    return extract<FootModelData>(RightFootDataIndex());
  }
};

class BipedModel : public ModelBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>,
                                    BipedModelData> {
 public:
  using Self = BipedModel;
  using Data = BipedModelData;
  using Solver = RungeKutta4<std::array<Vec3D, 2>>;
  using Base = ModelBase<Vec3D, Solver, Data>;

  using TrunkModel = ComZmpModel;
  using FootModel = BipedFootModel;
  using TrunkModelPtr = std::shared_ptr<TrunkModel>;
  using FootModelPtr = std::shared_ptr<FootModel>;
  using TrunkModelData = ComZmpModelData;
  using FootModelData = BipedFootModelData;

 public:
  BipedModel();
  BipedModel(const Vec3D& t_com_position, double t_mass, double t_foot_dist);
  BipedModel(const Vec3D& t_com_position, double t_mass,
             const Vec3D& t_left_foot_position,
             const Vec3D& t_right_foot_position);
  explicit BipedModel(Data t_data);
  BipedModel(Data t_data, TrunkModelPtr t_trunk_ptr,
             FootModelPtr t_left_foot_ptr, FootModelPtr t_right_foot_ptr);
  virtual ~BipedModel() = default;

  // accessors
  const TrunkModel& trunk() const { return *m_trunk_ptr; }
  const FootModel& left_foot() const { return *m_left_foot_ptr; }
  const FootModel& right_foot() const { return *m_right_foot_ptr; }
  TrunkModel& trunk() { return *m_trunk_ptr; }
  FootModel& left_foot() { return *m_left_foot_ptr; }
  FootModel& right_foot() { return *m_right_foot_ptr; }
  const TrunkModelPtr& trunk_ptr() const { return m_trunk_ptr; }
  const FootModelPtr& left_foot_ptr() const { return m_left_foot_ptr; }
  const FootModelPtr& right_foot_ptr() const { return m_right_foot_ptr; }

  // reset
  virtual Self& reset() override;
  virtual Self& reset(const Vec3D& t_com_position, double t_foot_dist);
  virtual Self& reset(const Vec3D& t_com_position,
                      const Vec3D& t_left_foot_position,
                      const Vec3D& t_right_foot_position);

 private:
  TrunkModelPtr m_trunk_ptr;
  FootModelPtr m_left_foot_ptr;
  FootModelPtr m_right_foot_ptr;

  TrunkModelData extract_trunk_model_data() const {
    return data().extract<TrunkModelData>(Data::TrunkDataIndex());
  }
  FootModelData extract_left_foot_model_data() const {
    return data().extract<FootModelData>(Data::LeftFootDataIndex());
  }
  FootModelData extract_right_foot_model_data() const {
    return data().extract<FootModelData>(Data::RightFootDataIndex());
  }
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_MODEL_HPP_
