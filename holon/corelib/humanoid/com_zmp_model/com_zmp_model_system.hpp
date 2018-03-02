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

#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_data.hpp"

namespace holon {

class ComZmpModelSystem {
  using Data = ComZmpModelData;
  using DataPtr = ComZmpModelDataPtr;
  using self_ref = ComZmpModelSystem&;

 public:
  using State = std::array<Vec3D, 2>;
  using Function =
      std::function<Vec3D(const Vec3D&, const Vec3D&, const double)>;

 public:
  explicit ComZmpModelSystem(DataPtr t_data_ptr);

  // special member functions
  virtual ~ComZmpModelSystem() noexcept = default;
  ComZmpModelSystem(const ComZmpModelSystem&) = default;
  ComZmpModelSystem(ComZmpModelSystem&&) noexcept = delete;
  ComZmpModelSystem& operator=(const ComZmpModelSystem&) = delete;
  ComZmpModelSystem& operator=(ComZmpModelSystem&&) noexcept = delete;

  // operator()
  void operator()(const State& x, State& dxdt, const double t) const;

  // accessors
  inline DataPtr data_ptr() const { return m_data_ptr; }
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
  inline bool isZmpPositionSet() const {
    return static_cast<bool>(m_zmp_position_f);
  }

  // mutators
  self_ref set_data_ptr(DataPtr t_data_ptr);
  self_ref set_com_acceleration_f(Function t_com_acceleration_f);
  self_ref set_reaction_force_f(Function t_reaction_force_f);
  self_ref set_external_force_f(Function t_external_force_f);
  self_ref set_zmp_position_f(Function t_zmp_position_f);

  Function getDefaultComAccFunc() const;
  Function getComAccFuncWithReactForce() const;
  Function getComAccFuncWithZmpPos() const;
  Function getDefaultReactForceFunc() const;
  Function getDefaultExtForceFunc() const;
  Function getDefaultZmpPosFunc() const;

 private:
  DataPtr m_data_ptr;
  Function m_com_acceleration_f;
  Function m_reaction_force_f;
  Function m_external_force_f;
  Function m_zmp_position_f;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_SYSTEM_HPP_
