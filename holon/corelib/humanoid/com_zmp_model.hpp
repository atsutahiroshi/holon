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

#include <zeo/zeo_vec3d.h>

namespace holon {

class ComZmpModelData {
  static constexpr double default_mass = 1;
  static constexpr zVec3D default_com_position = {{0, 0, 1}};

 public:
  ComZmpModelData();
  explicit ComZmpModelData(double t_mass);

  // special member functions
  virtual ~ComZmpModelData() = default;
  ComZmpModelData(const ComZmpModelData&) = default;
  ComZmpModelData(ComZmpModelData&&) = default;
  ComZmpModelData& operator=(const ComZmpModelData&) = default;
  ComZmpModelData& operator=(ComZmpModelData&&) = default;

  // accessors
  inline double mass() const noexcept { return m_mass; }
  inline const zVec3D com_position() const noexcept { return m_com_position; }
  inline const zVec3D com_velocity() const noexcept { return m_com_velocity; }
  inline const zVec3D com_acceleration() const noexcept {
    return m_com_acceleration;
  }
  inline const zVec3D zmp_position() const noexcept { return m_zmp_position; }

  // mutators
  ComZmpModelData& set_mass(double t_mass);
  ComZmpModelData& set_com_position(const zVec3D& t_com_position);
  ComZmpModelData& set_com_velocity(const zVec3D& t_com_velocity);
  ComZmpModelData& set_com_acceleration(const zVec3D& t_com_acceleration);
  ComZmpModelData& set_zmp_position(const zVec3D& t_zmp_position);
  ComZmpModelData& reset(const zVec3D& t_com_position);

 private:
  double m_mass;
  zVec3D m_com_position;
  zVec3D m_com_velocity;
  zVec3D m_com_acceleration;
  zVec3D m_zmp_position;
};

class ComZmpModel {
  static constexpr double default_time_step = 0.001;

 public:
  ComZmpModel();
  ComZmpModel(double t_mass);

  // special member functions
  virtual ~ComZmpModel() = default;
  ComZmpModel(const ComZmpModel&) = delete;
  ComZmpModel(ComZmpModel&&) = delete;
  ComZmpModel& operator=(const ComZmpModel&) = delete;
  ComZmpModel& operator=(ComZmpModel&&) = delete;

  // accessors
  inline double mass() const noexcept { return m_data.mass(); }
  inline const zVec3D com_position() const noexcept {
    return m_data.com_position();
  }
  inline const zVec3D com_velocity() const noexcept {
    return m_data.com_velocity();
  }
  inline const zVec3D com_acceleration() const noexcept {
    return m_data.com_acceleration();
  }
  inline const zVec3D zmp_position() const noexcept {
    return m_data.zmp_position();
  }
  inline double time_step() const noexcept { return m_time_step; }

  // mutators
  ComZmpModel& set_mass(double t_mass);
  ComZmpModel& set_com_position(const zVec3D& t_com_position);
  ComZmpModel& set_com_velocity(const zVec3D& t_com_velocity);
  ComZmpModel& set_com_acceleration(const zVec3D& t_com_acceleration);
  ComZmpModel& set_zmp_position(const zVec3D& t_zmp_position);
  ComZmpModel& set_time_step(double t_time_step);
  ComZmpModel& reset(const zVec3D& t_com_position);

  double computeZetaSqr(const zVec3D& com_position) const;
  double computeZeta(const zVec3D& com_position) const;

  zVec3D computeAcceleration(const zVec3D& com_position,
                             const zVec3D& zmp_position) const;

  bool update();
  bool update(double t_time_step);

 private:
  ComZmpModelData m_data;
  double m_time_step;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
