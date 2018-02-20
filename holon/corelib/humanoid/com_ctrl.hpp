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
#include "holon/corelib/humanoid/com_ctrl_x.hpp"
#include "holon/corelib/humanoid/com_ctrl_y.hpp"
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

  void clear();
  void set_com_position(const Vec3D& t_com_position);
  void set_com_position(opt_double t_xd, opt_double t_yd, opt_double t_zd);
  void set_com_velocity(opt_double t_vxd, opt_double t_vyd);
};

struct ComCtrlInputs {
  Vec3D com_position;
  Vec3D com_velocity;
  double qx1, qx2;
  double qy1, qy2;
  double qz1, qz2;
};

struct ComCtrlOutputs {
  Vec3D com_position;
  Vec3D com_velocity;
  Vec3D com_acceleration;
  Vec3D zmp_position;
  Vec3D reaction_force;
  double zeta;
};

class ComCtrl {
 public:
  using Model = ComZmpModel;
  using States = ComZmpModelData;
  using Inputs = ComCtrlInputs;
  using Outputs = ComCtrlOutputs;
  using UserCommands = ComCtrlCommands;
  using StatesPtr = std::shared_ptr<States>;
  using InputsPtr = std::shared_ptr<Inputs>;
  using OutputsPtr = std::shared_ptr<Outputs>;
  using UserCommandsPtr = std::shared_ptr<UserCommands>;

  // constructors
  ComCtrl();

  // special member functions
  virtual ~ComCtrl() = default;
  ComCtrl(const ComCtrl&) = delete;
  ComCtrl(ComCtrl&&) = delete;
  ComCtrl& operator=(const ComCtrl&) = delete;
  ComCtrl& operator=(ComCtrl&&) = delete;

  // accessors
  inline ComCtrlX& x() noexcept { return m_x; }
  inline ComCtrlY& y() noexcept { return m_y; }
  inline States& states() noexcept { return *m_states_ptr; }
  // const accessors
  inline const Vec3D& initial_com_position() const noexcept {
    return m_initial_com_position;
  }
  inline const ComCtrlX& x() const noexcept { return m_x; }
  inline const ComCtrlY& y() const noexcept { return m_y; }
  inline const Model& model() const noexcept { return m_model; }
  inline const States& states() const noexcept { return *m_states_ptr; }
  inline const Inputs& inputs() const noexcept { return *m_inputs_ptr; }
  inline const Outputs& outputs() const noexcept { return *m_outputs_ptr; }
  inline const UserCommands& cmds() const noexcept { return *m_user_cmds_ptr; }
  inline double time_step() const noexcept { return model().time_step(); }

  // mutators
  ComCtrl& set_time_step(double t_time_step);
  ComCtrl& reset(const Vec3D& t_com_position);

  //
  inline UserCommandsPtr getUserCommands() const noexcept {
    return m_user_cmds_ptr;
  }

  // computing functions
  Vec3D computeDesReactForce() const;
  double computeDesZeta(const Vec3D& t_reaction_force) const;
  Vec3D computeDesZmpPos(const Vec3D& t_ref_com_position,
                         const Vec3D& t_com_position,
                         const Vec3D& t_com_velocity,
                         double t_desired_zeta) const;

  // update functions
  bool update();
  bool update(double t_time_step);

 private:
  ComCtrlX m_x;
  ComCtrlY m_y;
  Model m_model;
  StatesPtr m_states_ptr;
  InputsPtr m_inputs_ptr;
  OutputsPtr m_outputs_ptr;
  UserCommandsPtr m_user_cmds_ptr;
  Vec3D m_initial_com_position;

  void updateCtrlParam();
  void remapUserCommandsToInputs();
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_HPP_
