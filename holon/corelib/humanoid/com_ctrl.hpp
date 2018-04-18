/* com_ctrl - COM Controller
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

#ifndef HOLON_HUMANOID_COM_CTRL_HPP_
#define HOLON_HUMANOID_COM_CTRL_HPP_

#include <memory>
#include "holon/corelib/common/types.hpp"
#include "holon/corelib/control/ctrl_base.hpp"
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/humanoid/com_zmp_model.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

struct ComCtrlCommandsRawData : RawDataBase {
  using opt_double = optional<double>;

  opt_double xd, yd, zd;
  opt_double vxd, vyd;
  opt_double qx1, qx2;
  opt_double qy1, qy2;
  opt_double qz1, qz2;
  opt_double rho, dist, kr;
  opt_double vhp;

  ComCtrlCommandsRawData() = default;
  void clear();
  void set_com_position(const Vec3D& t_com_position);
  void set_com_position(opt_double t_xd, opt_double t_yd, opt_double t_zd);
  void set_com_velocity(opt_double t_vxd, opt_double t_vyd);
};

struct ComCtrlParamsRawData : RawDataBase {
  Vec3D com_position;
  Vec3D com_velocity;
  double mass;
  double qx1, qx2;
  double qy1, qy2;
  double qz1, qz2;
  double rho, dist, kr;
  double vhp;
  ComCtrlParamsRawData();
  ComCtrlParamsRawData(const Vec3D& t_com_position, double t_mass);
};

struct ComCtrlOutputsRawData : RawDataBase {
  Vec3D com_position;
  Vec3D com_velocity;
  Vec3D com_acceleration;
  Vec3D zmp_position;
  Vec3D reaction_force;
  ComCtrlOutputsRawData();
};

class ComCtrlData
    : public DataSetBase<ComZmpModelRawData, ComCtrlParamsRawData,
                         ComCtrlOutputsRawData, ComCtrlCommandsRawData> {
  HOLON_DEFINE_DEFAULT_DATA_CTOR(ComCtrlData);

 public:
  using ModelRawData = ComZmpModelRawData;
  using ParamsRawData = ComCtrlParamsRawData;
  using OutputsRawData = ComCtrlOutputsRawData;
  using CommandsRawData = ComCtrlCommandsRawData;
  using ModelDataIndex = index_seq<0>;
  using ParamsDataIndex = index_seq<1>;
  using OutputsDataIndex = index_seq<2>;
  using CommandsDataIndex = index_seq<3>;

  ComCtrlData(const Vec3D& t_com_position = ModelRawData::default_com_position,
              double t_mass = ModelRawData::default_mass);
};

class ComCtrl : public CtrlBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>,
                                ComCtrlData, ComZmpModel> {
  using Self = ComCtrl;
  using Base = CtrlBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>, ComCtrlData,
                        ComZmpModel>;

 public:
  using Model = ComZmpModel;
  using Data = ComCtrlData;

  ComCtrl();
  explicit ComCtrl(const Model& t_model);
  explicit ComCtrl(Data t_data);
  ComCtrl(Data t_data, std::shared_ptr<Model> t_model_ptr);
  virtual ~ComCtrl() = default;

  // accessors
  inline const Data::CommandsRawData& commands() const noexcept {
    return data().get<Data::CommandsDataIndex::get<0>()>();
  }
  inline Vec3D initial_com_position() const noexcept {
    return model().initial_com_position();
  }
  inline Vec3D default_com_position() const noexcept {
    return m_default_com_position;
  }
  inline double canonical_foot_dist() const noexcept {
    return m_canonical_foot_dist;
  }

  // mutators
  Self& set_canonical_foot_dist(double t_canonical_foot_dist);
  virtual Self& reset() override;
  virtual Self& reset(const Vec3D& t_com_position);
  virtual Self& reset(const Vec3D& t_com_position, double t_foot_dist);

  //
  std::shared_ptr<ComCtrlCommandsRawData> getCommands() const noexcept {
    return data().get_ptr<Data::CommandsDataIndex::get<0>()>();
  }

  // update functions
  void feedback(const Model& t_model);
  void feedback(ComZmpModelData t_model_data);
  void feedback(const Vec3D& t_com_position, const Vec3D& t_com_velocity);
  virtual bool update() override;
  virtual bool update(double t_time_step) override;

  using CallbackFunc = ComZmpModel::CallbackFunc;
  Vec3D computeDesReactForce(const Vec3D& t_com_position,
                             const Vec3D& t_com_velocity, const double t);
  Vec3D computeDesZmpPos(const Vec3D& t_com_position,
                         const Vec3D& t_com_velocity, const double t);
  CallbackFunc getReactionForceCallback();
  CallbackFunc getZmpPositionCallback();

  double phaseLF() const;
  double phaseRF() const;

 private:
  Vec3D m_default_com_position;
  double m_canonical_foot_dist;
  double m_max_foot_dist;
  double m_current_foot_dist;

  void init();
  void updateSideward();
  void updateParams();
  void updateOutputs();
  void updateDefaultComPosition();
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_HPP_
