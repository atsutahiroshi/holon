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

#include <memory>
#include "holon/corelib/common/optional.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

struct ComZmpModelData {
  double mass;
  Vec3D nu;
  Vec3D com_position;
  Vec3D com_velocity;
  Vec3D com_acceleration;
  Vec3D zmp_position;
  Vec3D reaction_force;
  Vec3D external_force;

  // constructors
  ComZmpModelData();
  explicit ComZmpModelData(double t_mass);

 private:
  static const double default_mass;
  static const Vec3D default_com_position;
};

std::shared_ptr<ComZmpModelData> ComZmpModelDataFactory();
std::shared_ptr<ComZmpModelData> ComZmpModelDataFactory(double t_mass);

class ComZmpModel {
  static constexpr double default_time_step = 0.001;

 public:
  using Data = ComZmpModelData;
  using DataPtr = std::shared_ptr<ComZmpModelData>;

  ComZmpModel();
  explicit ComZmpModel(double t_mass);
  explicit ComZmpModel(DataPtr t_data_ptr);

  // special member functions
  virtual ~ComZmpModel() = default;
  ComZmpModel(const ComZmpModel&) = delete;
  ComZmpModel(ComZmpModel&&) = delete;
  ComZmpModel& operator=(const ComZmpModel&) = delete;
  ComZmpModel& operator=(ComZmpModel&&) = delete;

  // accessors
  inline const Data& data() const noexcept { return *m_data_ptr; }
  inline const DataPtr& data_ptr() const noexcept { return m_data_ptr; }
  inline double time_step() const noexcept { return m_time_step; }
  inline double mass() const noexcept { return data().mass; }

  // mutators
  ComZmpModel& set_data_ptr(DataPtr t_data_ptr);
  ComZmpModel& set_external_force(const Vec3D& t_external_force);
  ComZmpModel& clear_external_force() { return set_external_force(kVec3DZero); }
  ComZmpModel& set_time_step(double t_time_step);
  ComZmpModel& reset(const Vec3D& t_com_position);

  // copy data
  void copy_data(const ComZmpModel& t_model);
  void copy_data(const Data& t_data);

  // functions to compute squared zeta
  double computeSqrZeta(double t_com_position_z, double t_zmp_position_z,
                        double t_com_acceleration_z) const;
  double computeSqrZeta(double t_com_position_z, double t_zmp_position_z,
                        double t_reation_force_z, double t_mass) const;
  double computeSqrZeta(const Vec3D& t_com_position,
                        const Vec3D& t_zmp_position,
                        const Vec3D& t_com_acceleration,
                        const Vec3D& t_nu = kVec3DZ) const;
  double computeSqrZeta(const Vec3D& t_com_position,
                        const Vec3D& t_zmp_position,
                        const Vec3D& t_reaction_force, double t_mass,
                        const Vec3D& t_nu = kVec3DZ) const;

  // functions to compute zeta
  double computeZeta(double t_com_position_z, double t_zmp_position_z,
                     double t_com_acceleration_z) const;
  double computeZeta(double t_com_position_z, double t_zmp_position_z,
                     double t_reation_force_z, double t_mass) const;
  double computeZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                     const Vec3D& t_com_acceleration,
                     const Vec3D& t_nu = kVec3DZ) const;
  double computeZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                     const Vec3D& t_reaction_force, double t_mass,
                     const Vec3D& t_nu = kVec3DZ) const;

  // functions to compute reaction force
  Vec3D computeReactForce(const Vec3D& t_com_acceleration, double t_mass) const;
  Vec3D computeReactForce(const Vec3D& t_com_position,
                          const Vec3D& t_zmp_position, double t_sqr_zeta,
                          double t_mass) const;
  Vec3D computeReactForce(const Vec3D& t_com_position,
                          const Vec3D& t_zmp_position,
                          const Vec3D& t_com_acceleration, double t_mass,
                          const Vec3D& t_nu = kVec3DZ) const;
  Vec3D computeReactForce(const Vec3D& t_com_position,
                          const Vec3D& t_zmp_position,
                          double t_reaction_force_z) const;

  // functions to compute COM acceleration
  Vec3D computeComAcc(const Vec3D& t_reaction_force, double t_mass,
                      const Vec3D& t_external_force = kVec3DZero) const;
  Vec3D computeComAcc(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                      double t_sqr_zeta, double t_mass = 1,
                      const Vec3D& t_external_force = kVec3DZero) const;
  Vec3D computeComAcc(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                      const Vec3D& t_reaction_force, double t_mass,
                      const Vec3D& t_external_force = kVec3DZero,
                      const Vec3D& t_nu = kVec3DZ) const;

  // update functions
  void inputZmpPos(const Vec3D& t_zmp_position,
                   optional<double> t_reaction_force_z = nullopt);
  void inputReactForce(const Vec3D& t_reaction_force);
  void inputComAcc(const Vec3D& t_com_acceleration);

  bool update();
  bool update(double t_time_step);

 private:
  DataPtr m_data_ptr;
  double m_time_step;

  bool isTimeStepValid(double t_time_step) const;
  bool isMassValid(double t_mass) const;
  bool isComZmpDiffValid(double t_com_position_z,
                         double t_zmp_position_z) const;
  bool isComZmpDiffValid(const Vec3D& t_com_position,
                         const Vec3D& t_zmp_position,
                         const Vec3D& t_nu = kVec3DZ) const;
  bool isReactionForceValid(double t_reaction_force_z) const;
  bool isReactionForceValid(const Vec3D& t_reaction_force,
                            const Vec3D& t_nu = kVec3DZ) const;
  bool isComAccelerationValid(double t_com_acceleration_z) const;
  bool isComAccelerationValid(const Vec3D& t_com_acceleration,
                              const Vec3D& t_nu = kVec3DZ) const;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
