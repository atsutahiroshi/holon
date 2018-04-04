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
  using Self = BipedModelData;
  using Base = DataSetBase<ComZmpModelRawData, PointMassModelRawData<Vec3D>,
                           PointMassModelRawData<Vec3D>>;
  using TrunkRawDataType = ComZmpModelRawData;
  using FootRawDataType = PointMassModelRawData<Vec3D>;

 public:
  BipedModelData() = default;
  BipedModelData(const TrunkRawDataType& t_trunk_raw_data,
                 const FootRawDataType& t_lf_raw_data,
                 const FootRawDataType& t_rf_raw_data) {}
  BipedModelData(std::shared_ptr<TrunkRawDataType> t_trunk_raw_data_ptr,
                 std::shared_ptr<FootRawDataType> t_lf_raw_data_ptr,
                 std::shared_ptr<FootRawDataType> t_rf_raw_data_ptr) {}
  BipedModelData(const Vec3D& t_com_position, double t_mass,
                 double t_foot_dist) {}
  BipedModelData(const Vec3D& t_com_position, double t_mass,
                 const Vec3D& t_lfoot_position, const Vec3D& t_rfoot_position) {
  }
  virtual ~BipedModelData() = default;

  TrunkRawDataType& trunk() { return get<0>(); }
  FootRawDataType& left_foot() { return get<1>(); }
  FootRawDataType& right_foot() { return get<2>(); }
};

class BipedModelSystem : public SystemBase<Vec3D, BipedModelData> {
  using Data = BipedModelData;
  using Self = BipedModelSystem;

 public:
  using StateArray = std::array<Vec3D, 2>;
  using Function =
      std::function<Vec3D(const Vec3D&, const Vec3D&, const double)>;

  BipedModelSystem(Data t_data) : SystemBase(t_data) {}
  virtual ~BipedModelSystem() = default;

  virtual StateArray operator()(const StateArray& state,
                                const double t) const override {}
};

class BipedModel : public ModelBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>,
                                    BipedModelData, BipedModelSystem> {
  using Self = BipedModel;
  using Base = ModelBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>,
                         BipedModelData, BipedModelSystem>;

 public:
  using Data = BipedModelData;
  using System = BipedModelSystem;
  using Function = System::Function;
  using TrunkRawDataType = ComZmpModelRawData;
  using FootRawDataType = PointMassModelRawData<Vec3D>;

 public:
  BipedModel() = default;
  BipedModel(const Vec3D& t_com_position, double t_mass, double t_foot_dist) {}
  BipedModel(const Vec3D& t_com_position, double t_mass,
             const Vec3D& t_lfoot_position, const Vec3D& t_rfoot_position) {}
  explicit BipedModel(Data t_data) {}
  virtual ~BipedModel() = default;

  // accessors
  const TrunkRawDataType& trunk() const;
  const FootRawDataType& lfoot() const;
  const FootRawDataType& rfoot() const;
  TrunkRawDataType& trunk();
  FootRawDataType& lfoot();
  FootRawDataType& rfoot();
  virtual Self& reset() override {}

  // update
  virtual bool update() override {}
  virtual bool update(double dt) override {}
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_MODEL_HPP_
