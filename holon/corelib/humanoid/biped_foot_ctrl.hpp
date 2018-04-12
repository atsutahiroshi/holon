/* biped_foot_ctrl - Bipedal foot control
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

#ifndef HOLON_HUMANOID_BIPED_FOOT_CTRL_HPP_
#define HOLON_HUMANOID_BIPED_FOOT_CTRL_HPP_

#include "holon/corelib/common/optional.hpp"
#include "holon/corelib/control/pd_ctrl.hpp"
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/humanoid/biped_foot_model.hpp"

namespace holon {

struct BipedFootCtrlParamsRawData : PdCtrlParamsRawData<Vec3D> {
  double max_height = 0;
  BipedFootCtrlParamsRawData() = default;
};
struct BipedFootCtrlOutputsRawData : PdCtrlOutputsRawData<Vec3D> {
  BipedFootCtrlOutputsRawData() = default;
};
struct BipedFootCtrlCommandsRawData : RawDataBase {
  optional<double> max_height;
  std::array<optional<double>, 3> stiffness;
  std::array<optional<double>, 3> damping;
};

class BipedFootCtrlData
    : public DataSetBase<
          PointMassModelRawData<Vec3D>, BipedFootCtrlParamsRawData,
          BipedFootCtrlOutputsRawData, BipedFootCtrlCommandsRawData> {
  HOLON_DEFINE_DEFAULT_DATA_CTOR(BipedFootCtrlData);
  using Base =
      DataSetBase<BipedFootModelData, BipedFootCtrlParamsRawData,
                  BipedFootCtrlOutputsRawData, BipedFootCtrlCommandsRawData>;

 public:
  using ModelRawData = BipedFootModelData;
  using ParamsRawData = BipedFootCtrlParamsRawData;
  using OutputsRawData = BipedFootCtrlOutputsRawData;
  using CommandsRawData = BipedFootCtrlCommandsRawData;
  using ModelDataIndex = index_seq<0>;
  using ParamsDataIndex = index_seq<1>;
  using OutputsDataIndex = index_seq<2>;
  using CommandsDataIndex = index_seq<3>;

  BipedFootCtrlData(const Vec3D& t_initial_position);
};

class BipedFootCtrl : CtrlBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>,
                               BipedFootCtrlData, BipedFootModel> {};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_FOOT_CTRL_HPP_
