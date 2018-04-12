/* biped_foot_model - Bipedal foot model
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

#ifndef HOLON_HUMANOID_BIPED_FOOT_MODEL_HPP_
#define HOLON_HUMANOID_BIPED_FOOT_MODEL_HPP_

#include "holon/corelib/control/point_mass_model.hpp"
#include "holon/corelib/math/ode_runge_kutta4.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

class BipedFootModelData : public PointMassModelData<Vec3D> {
 public:
  template <typename... Args>
  explicit BipedFootModelData(Args... args)
      : PointMassModelData<Vec3D>(args...) {}
};

enum class BipedFootType : int { none = 0, left = 1, right = -1 };

class BipedFootModel
    : public PointMassModel<
          Vec3D, std::array<Vec3D, 2>, RungeKutta4<std::array<Vec3D, 2>>,
          BipedFootModelData, PointMassModelSystem<Vec3D, BipedFootModelData>> {
  using Self = BipedFootModel;
  using Base =
      PointMassModel<Vec3D, std::array<Vec3D, 2>,
                     RungeKutta4<std::array<Vec3D, 2>>, BipedFootModelData,
                     PointMassModelSystem<Vec3D, BipedFootModelData>>;
  using Data = BipedFootModelData;

 public:
  explicit BipedFootModel(BipedFootType t_type = BipedFootType::none);
  explicit BipedFootModel(const Vec3D& t_initial_position,
                          BipedFootType t_type = BipedFootType::none);
  explicit BipedFootModel(Data t_data,
                          BipedFootType t_type = BipedFootType::none);
  virtual ~BipedFootModel() = default;

  BipedFootType type() { return m_type; }
  int dir() { return static_cast<int>(m_type); }

 private:
  BipedFootType m_type;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_FOOT_MODEL_HPP_
