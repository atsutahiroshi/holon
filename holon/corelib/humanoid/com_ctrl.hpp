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
  optional<Vec3D> com_position;
  optional<Vec3D> com_velocity;
  optional<double> qx1, qx2;
  optional<double> qy1, qy2;
  optional<double> qz1, qz2;
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
  double zeta;
};

class ComCtrl {
 public:
  typedef std::shared_ptr<ComCtrlCommands> UserCommands;
  typedef ComCtrlInputs Inputs;
  typedef ComCtrlOutputs Outputs;
  typedef ComZmpModelData States;
  typedef ComZmpModel Model;

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
  inline Model& model() noexcept { return m_model; }
  inline Inputs& inputs() noexcept { return m_inputs; }
  // const accessors
  inline const ComCtrlX& x() const noexcept { return m_x; }
  inline const ComCtrlY& y() const noexcept { return m_y; }
  inline const Model& model() const noexcept { return m_model; }
  inline const Inputs& inputs() const noexcept { return m_inputs; }
  inline const Outputs& outputs() const noexcept { return m_outputs; }
  inline double time_step() const noexcept { return model().time_step(); }

  // mutators
  ComCtrl& set_time_step(double t_time_step);

  //
  inline UserCommands getUserCommands() const noexcept { return m_user_cmds; }

  // computing functions
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

  UserCommands m_user_cmds;
  Inputs m_inputs;
  Outputs m_outputs;
  // States m_states;

  void remapUserCommandsToInputs();
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_HPP_
