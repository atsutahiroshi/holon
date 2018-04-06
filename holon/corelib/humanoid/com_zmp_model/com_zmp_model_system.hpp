/* com_zmp_model_system - Definition of the COM-ZMP model system
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

#ifndef HOLON_HUMANOID_COM_ZMP_MODEL_SYSTEM_HPP_
#define HOLON_HUMANOID_COM_ZMP_MODEL_SYSTEM_HPP_

#include "holon/corelib/control/system_base.hpp"
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_data.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

class ComZmpModelSystem : public SystemBase<Vec3D, ComZmpModelData> {
  using Self = ComZmpModelSystem;
  using Base = SystemBase<Vec3D, ComZmpModelData>;
  using Data = ComZmpModelData;

 public:
  using Function =
      std::function<Vec3D(const Vec3D&, const Vec3D&, const double)>;

 public:
  explicit ComZmpModelSystem(Data t_data);
  virtual ~ComZmpModelSystem() noexcept = default;

  // operator()
  virtual StateArray operator()(const StateArray& state,
                                const double t) const override;

  // accessors
  inline Vec3D com_acceleration(const Vec3D& p, const Vec3D& v,
                                const double t) const {
    return m_com_acceleration_f(p, v, t);
  }
  inline Vec3D reaction_force(const Vec3D& p, const Vec3D& v,
                              const double t) const {
    return m_reaction_force_f(p, v, t);
  }
  inline Vec3D external_force(const Vec3D& p, const Vec3D& v,
                              const double t) const {
    return m_external_force_f(p, v, t);
  }
  inline Vec3D zmp_position(const Vec3D& p, const Vec3D& v,
                            const double t) const {
    return m_zmp_position_f(p, v, t);
  }
  inline bool is_set_zmp_position() const {
    return static_cast<bool>(m_zmp_position_f);
  }

  // mutators
  Self& set_com_acceleration_f(Function t_com_acceleration_f);
  Self& set_reaction_force_f(Function t_reaction_force_f);
  Self& set_external_force_f(Function t_external_force_f);
  Self& set_zmp_position_f(Function t_zmp_position_f);

  Function getDefaultComAccFunc() const;
  Function getComAccFuncWithReactForce() const;
  Function getComAccFuncWithZmpPos() const;
  Function getDefaultReactForceFunc() const;
  Function getDefaultExtForceFunc() const;
  Function getDefaultZmpPosFunc() const;

 private:
  Function m_com_acceleration_f;
  Function m_reaction_force_f;
  Function m_external_force_f;
  Function m_zmp_position_f;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_SYSTEM_HPP_
