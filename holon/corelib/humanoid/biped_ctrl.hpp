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
                          // refs raw data
                          ComCtrlRefsRawData, PdCtrlRefsRawData<Vec3D>,
                          PdCtrlRefsRawData<Vec3D>,
                          // outputs raw data
                          ComCtrlOutputsRawData, PdCtrlOutputsRawData<Vec3D>,
                          PdCtrlOutputsRawData<Vec3D>,
                          // commands raw data
                          ComCtrlCommandsRawData> {
  HOLON_DEFINE_DEFAULT_DATA_CTOR(BipedCtrlData);

 public:
  using ModelDataIndex = index_seq<0, 1, 2>;
  using RefsDataIndex = index_seq<3, 4, 5>;
  using OutputsDataIndex = index_seq<6, 7, 8>;
  using CommandsDataIndex = index_seq<9>;
};

class BipedCtrl : public CtrlBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>,
                                  BipedCtrlData, BipedModel> {
  using Self = BipedCtrl;
  using Base = CtrlBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>, BipedCtrlData,
                        BipedModel>;

 public:
  using Model = BipedModel;
  using Data = BipedCtrlData;

 public:
  BipedCtrl();
  explicit BipedCtrl(Data t_data);
  explicit BipedCtrl(const Model& t_model);
  virtual ~BipedCtrl() = default;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_CTRL_HPP_
