/* com_zmp_model - COM-ZMP model
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

#ifndef HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
#define HOLON_HUMANOID_COM_ZMP_MODEL_HPP_

#include <array>
#include "holon/corelib/common/optional.hpp"
#include "holon/corelib/control/model_base.hpp"
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_data.hpp"
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_system.hpp"
#include "holon/corelib/math/ode_runge_kutta4.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

class ComZmpModel {
  static constexpr double default_time_step = 0.001;
  using Self = ComZmpModel;

 public:
  using Data = ComZmpModelData;
  using DataPtr = ComZmpModelDataPtr;
  using System = ComZmpModelSystem;
  using CallbackFunc = ComZmpModelSystem::Function;

  ComZmpModel();
  explicit ComZmpModel(const Vec3D& t_com_position);
  ComZmpModel(const Vec3D& t_com_position, double mass);
  explicit ComZmpModel(DataPtr t_data_ptr);

  // special member functions
  virtual ~ComZmpModel() = default;
  ComZmpModel(const ComZmpModel&) = delete;
  ComZmpModel(ComZmpModel&&) = delete;
  Self& operator=(const ComZmpModel&) = delete;
  Self& operator=(ComZmpModel&&) = delete;

  // accessors
  inline double time() const noexcept { return m_time; }
  inline double time_step() const noexcept { return m_time_step; }
  inline const Data& data() const noexcept { return *m_data_ptr; }
  inline const DataPtr& data_ptr() const noexcept { return m_data_ptr; }
  inline Vec3D initial_com_position() const noexcept {
    return m_initial_com_position;
  }
  inline double mass() const noexcept { return data().mass; }
  inline const System& system() const noexcept { return m_system; }

  // mutators
  Self& set_time_step(double t_time_step);
  Self& set_data_ptr(DataPtr t_data_ptr);
  Self& set_initial_com_position(const Vec3D& t_initial_com_position);
  Self& reset(const Vec3D& t_com_position);

  // copy data
  void copy_data(const ComZmpModel& t_model);
  void copy_data(const Data& t_data);

  // callback functions
  Self& setExternalForceCallback(CallbackFunc t_f);
  Self& setReactionForceCallback(CallbackFunc t_f);
  Self& setZmpPositionCallback(CallbackFunc t_f);
  Self& setComAccelerationCallback(CallbackFunc t_f);

  Self& setZmpPosition(const Vec3D& t_zmp_position,
                       optional<double> t_reaction_force_z = nullopt);
  Self& setReactionForce(const Vec3D& t_reaction_force);
  Self& setExternalForce(const Vec3D& t_external_force);
  Self& removeZmpPosition();
  Self& removeReactionForce();
  Self& removeExternalForce();

  bool update();
  bool update(double t_time_step);

 private:
  double m_time;
  double m_time_step;
  DataPtr m_data_ptr;
  Vec3D m_initial_com_position;
  System m_system;

  bool isTimeStepValid(double t_time_step) const;
  bool isUpdatable(const Vec3D& p, const Vec3D& v);
  void updateData(const Vec3D& p, const Vec3D& v);
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
