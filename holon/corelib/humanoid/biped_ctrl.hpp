/* biped_ctrl - Biped robot control
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

#ifndef HOLON_HUMANOID_BIPED_CTRL_HPP_
#define HOLON_HUMANOID_BIPED_CTRL_HPP_

#include "holon/corelib/control/pd_ctrl.hpp"
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/humanoid/biped_model.hpp"
#include "holon/corelib/humanoid/com_ctrl.hpp"

namespace holon {

class BipedCtrlData : public DataSetBase<
                          // model raw data
                          ComZmpModelRawData, PointMassModelRawData<Vec3D>,
                          PointMassModelRawData<Vec3D>,
                          // params raw data
                          ComCtrlParamsRawData, PdCtrlParamsRawData<Vec3D>,
                          PdCtrlParamsRawData<Vec3D>,
                          // outputs raw data
                          ComCtrlOutputsRawData, PdCtrlOutputsRawData<Vec3D>,
                          PdCtrlOutputsRawData<Vec3D>,
                          // commands raw data
                          ComCtrlCommandsRawData> {
  HOLON_DEFINE_DEFAULT_DATA_CTOR(BipedCtrlData);

 public:
  using ModelDataIndex = index_seq<0, 1, 2>;
  using ParamsDataIndex = index_seq<3, 4, 5>;
  using OutputsDataIndex = index_seq<6, 7, 8>;
  using CommandsDataIndex = index_seq<9>;
  using TrunkDataIndex = index_seq<0, 3, 6, 9>;
  using LeftFootDataIndex = index_seq<1, 4, 7>;
  using RightFootDataIndex = index_seq<2, 5, 8>;
};

class BipedCtrl : public CtrlBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>,
                                  BipedCtrlData, BipedModel> {
  using Self = BipedCtrl;
  using Solver = RungeKutta4<std::array<Vec3D, 2>>;
  using Base = CtrlBase<Vec3D, Solver, BipedCtrlData, BipedModel>;

 public:
  using Model = BipedModel;
  using Data = BipedCtrlData;
  using TrunkCtrlData = ComCtrlData;
  using FootCtrlData = PdCtrlData<Vec3D>;
  using TrunkCtrl = ComCtrl;
  using FootCtrl = PdCtrl<Vec3D>;

 public:
  BipedCtrl();
  explicit BipedCtrl(Data t_data);
  explicit BipedCtrl(const Model& t_model);
  virtual ~BipedCtrl() = default;

  // accessors
  const TrunkCtrl& trunk() const { return m_trunk; }
  const FootCtrl& left_foot() const { return m_left_foot; }
  const FootCtrl& right_foot() const { return m_right_foot; }
  TrunkCtrl& trunk() { return m_trunk; }
  FootCtrl& left_foot() { return m_left_foot; }
  FootCtrl& right_foot() { return m_right_foot; }

  // reset functions
  virtual Self& reset() override;
  virtual Self& reset(const Vec3D& t_com_position, double t_foot_dist);
  virtual Self& reset(const Vec3D& t_com_position,
                      const Vec3D& t_left_foot_position,
                      const Vec3D& t_right_foot_position);

  // update functions
  virtual bool update() override;
  virtual bool update(double t_time_step) override;

 private:
  TrunkCtrl m_trunk;
  FootCtrl m_left_foot;
  FootCtrl m_right_foot;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_CTRL_HPP_
