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
#include "holon/corelib/common/optional.hpp"
#include "holon/corelib/control/ctrl_base.hpp"
#include "holon/corelib/humanoid/com_zmp_model.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

struct ComCtrlCommands {
  using opt_double = optional<double>;

  opt_double xd, yd, zd;
  opt_double vxd, vyd;
  opt_double qx1, qx2;
  opt_double qy1, qy2;
  opt_double qz1, qz2;
  opt_double rho, dist, kr;
  opt_double vhp;

  void clear();
  void set_com_position(const Vec3D& t_com_position);
  void set_com_position(opt_double t_xd, opt_double t_yd, opt_double t_zd);
  void set_com_velocity(opt_double t_vxd, opt_double t_vyd);
};

struct ComCtrlRefs {
  Vec3D com_position;
  Vec3D com_velocity;
  double qx1, qx2;
  double qy1, qy2;
  double qz1, qz2;
  double rho, dist, kr;
  double vhp;

  ComCtrlRefs();
  explicit ComCtrlRefs(const ComZmpModelData& t_data);
  explicit ComCtrlRefs(const ComZmpModel& t_model);
};

struct ComCtrlOutputs {
  Vec3D com_position;
  Vec3D com_velocity;
  Vec3D com_acceleration;
  Vec3D zmp_position;
  Vec3D reaction_force;
};

using ComCtrlCommandsPtr = std::shared_ptr<ComCtrlCommands>;
using ComCtrlRefsPtr = std::shared_ptr<ComCtrlRefs>;
using ComCtrlOutputsPtr = std::shared_ptr<ComCtrlOutputs>;
ComCtrlCommandsPtr createComCtrlCommands();
ComCtrlRefsPtr createComCtrlRefs();
ComCtrlOutputsPtr createComCtrlOutputs();

class ComCtrl
    : public CtrlBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>, ComZmpModelData,
                      ComZmpModel, ComCtrlRefs, ComCtrlOutputs> {
  using Self = ComCtrl;
  using Base =
      CtrlBase<Vec3D, RungeKutta4<std::array<Vec3D, 2>>, ComZmpModelData,
               ComZmpModel, ComCtrlRefs, ComCtrlOutputs>;

 public:
  using Model = ComZmpModel;
  using States = ComZmpModelData;
  using Refs = ComCtrlRefs;
  using Outputs = ComCtrlOutputs;
  using Commands = ComCtrlCommands;
  using CommandsPtr = std::shared_ptr<Commands>;

  // constructors
  ComCtrl();
  explicit ComCtrl(const Model& t_model);
  virtual ~ComCtrl() = default;

  // accessors
  inline const Commands& commands() const noexcept { return *m_commands_ptr; }
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
  ComCtrl& set_canonical_foot_dist(double t_canonical_foot_dist);
  virtual ComCtrl& reset(const Vec3D& t_com_position);
  virtual ComCtrl& reset(const Vec3D& t_com_position, double t_foot_dist);

  //
  inline CommandsPtr getCommands() const noexcept { return m_commands_ptr; }

  // update functions
  void feedback(const Model& t_model);
  void feedback(const Model::DataPtr& t_data_ptr);
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
  CommandsPtr m_commands_ptr;
  Vec3D m_default_com_position;
  double m_canonical_foot_dist;
  double m_max_foot_dist;
  double m_current_foot_dist;

  void updateSideward();
  void updateRefs();
  void updateOutputs();
  void updateDefaultComPosition();
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_HPP_
