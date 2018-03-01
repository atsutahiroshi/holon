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
#include <functional>
#include <memory>
#include <utility>
#include "holon/corelib/common/optional.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

struct ComZmpModelData {
 private:
  static const double default_mass;
  static const Vec3D default_com_position;

 public:
  double mass;
  Vec3D nu;
  Vec3D com_position;
  Vec3D com_velocity;
  Vec3D com_acceleration;
  Vec3D zmp_position;
  Vec3D reaction_force;
  Vec3D external_force;
  Vec3D total_force;

  // constructors
  ComZmpModelData(const Vec3D& t_com_position = default_com_position,
                  double t_mass = default_mass);
};

using ComZmpModelDataPtr = std::shared_ptr<ComZmpModelData>;
ComZmpModelDataPtr createComZmpModelData();
ComZmpModelDataPtr createComZmpModelData(const Vec3D& t_com_position);
ComZmpModelDataPtr createComZmpModelData(const Vec3D& t_com_position,
                                         double t_mass);

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
  inline DataPtr data_ptr() { return m_data_ptr; }
  Vec3D com_acceleration(const Vec3D& p, const Vec3D& v, const double t) {
    return m_com_acceleration_f(p, v, t);
  }
  Vec3D reaction_force(const Vec3D& p, const Vec3D& v, const double t) {
    return m_reaction_force_f(p, v, t);
  }
  Vec3D external_force(const Vec3D& p, const Vec3D& v, const double t) {
    return m_external_force_f(p, v, t);
  }
  Vec3D zmp_position(const Vec3D& p, const Vec3D& v, const double t) {
    return m_zmp_position_f(p, v, t);
  }
  inline bool isZmpPositionSet() { return static_cast<bool>(m_zmp_position_f); }

  // mutators
  self_ref set_data_ptr(DataPtr t_data_ptr);
  self_ref set_com_acceleration_f(Function t_com_acceleration_f);
  self_ref set_reaction_force_f(Function t_reaction_force_f);
  self_ref set_external_force_f(Function t_external_force_f);
  self_ref set_zmp_position_f(Function t_zmp_position_f);

  Function getDefaultComAccFunc();
  Function getComAccFuncWithReactForce();
  Function getComAccFuncWithZmpPos();
  Function getDefaultReactForceFunc();
  Function getDefaultExtForceFunc();
  Function getDefaultZmpPosFunc();

 private:
  DataPtr m_data_ptr;
  Function m_com_acceleration_f;
  Function m_reaction_force_f;
  Function m_external_force_f;
  Function m_zmp_position_f;
};

class ComZmpModel {
  static constexpr double default_time_step = 0.001;
  using self_ref = ComZmpModel&;

 public:
  using Data = ComZmpModelData;
  using DataPtr = ComZmpModelDataPtr;
  using System = ComZmpModelSystem;
  // using CallbackFunc = std::function<Vec3D(double, const Vec3D&, const
  // Vec3D&)>;
  using CallbackFunc = ComZmpModelSystem::Function;

  ComZmpModel();
  explicit ComZmpModel(const Vec3D& t_com_position);
  ComZmpModel(const Vec3D& t_com_position, double mass);
  explicit ComZmpModel(DataPtr t_data_ptr);

  // special member functions
  virtual ~ComZmpModel() = default;
  ComZmpModel(const ComZmpModel&) = delete;
  ComZmpModel(ComZmpModel&&) = delete;
  self_ref operator=(const ComZmpModel&) = delete;
  self_ref operator=(ComZmpModel&&) = delete;

  // accessors
  inline const Data& data() const noexcept { return *m_data_ptr; }
  inline const DataPtr& data_ptr() const noexcept { return m_data_ptr; }
  inline double time_step() const noexcept { return m_time_step; }
  inline Vec3D initial_com_position() const noexcept {
    return m_initial_com_position;
  }
  inline double mass() const noexcept { return data().mass; }

  // mutators
  self_ref set_data_ptr(DataPtr t_data_ptr);
  self_ref set_external_force(const Vec3D& t_external_force);
  self_ref clear_external_force() { return set_external_force(kVec3DZero); }
  self_ref set_time_step(double t_time_step);
  self_ref set_initial_com_position(const Vec3D& t_initial_com_position);
  self_ref reset(const Vec3D& t_com_position);

  // copy data
  void copy_data(const ComZmpModel& t_model);
  void copy_data(const Data& t_data);

  // update functions
  void inputZmpPos(const Vec3D& t_zmp_position,
                   optional<double> t_reaction_force_z = nullopt);
  void inputReactForce(const Vec3D& t_reaction_force);
  void inputComAcc(const Vec3D& t_com_acceleration);

  // callback functions
  self_ref setExternalForceCallback(CallbackFunc t_f);
  self_ref setReactionForceCallback(CallbackFunc t_f);
  self_ref setZmpPositionCallback(CallbackFunc t_f);
  self_ref setComAccelerationCallback(CallbackFunc t_f);

  self_ref setZmpPos(const Vec3D& t_zmp_position,
                     optional<double> t_reaction_force_z = nullopt);
  self_ref setExternalForce(const Vec3D& t_external_force);

  bool update();
  bool update(double t_time_step);

 private:
  DataPtr m_data_ptr;
  Vec3D m_initial_com_position;
  double m_time_step;
  System m_system;

  bool isTimeStepValid(double t_time_step) const;

  // CallbackFunc m_external_force_f = nullptr;
  // CallbackFunc m_reaction_force_f = nullptr;
  // CallbackFunc m_zmp_position_f = nullptr;
  // CallbackFunc m_com_acceleration_f = nullptr;

  // CallbackFunc getDefaultExternalForceUpdater();
  // CallbackFunc getDefaultReactionForceUpdater();
  // CallbackFunc getComAccUpdaterViaZmp();
  // CallbackFunc getComAccUpdaterViaForce();
  // CallbackFunc getComAccUpdater();
  // Vec3D updateComAccWithZmp(double t, const Vec3D& p, const Vec3D& v);
  // Vec3D updateComAccWithReactForce(double t, const Vec3D& p, const Vec3D& v);

  // std::pair<Vec3D, Vec3D> rk4_cat(std::pair<Vec3D, Vec3D> x, double dt,
  //                                 std::pair<Vec3D, Vec3D> dx);
  // std::pair<Vec3D, Vec3D> rk4_f(double t, std::pair<Vec3D, Vec3D> x);
  // std::pair<Vec3D, Vec3D> updateRk4(double t, std::pair<Vec3D, Vec3D> x,
  //                                   double dt);
  // std::pair<Vec3D, Vec3D> updateEuler(double t, std::pair<Vec3D, Vec3D> x,
  //                                     double dt);
};

namespace ComZmpModelFormula {

// functions to compute squared zeta
double computeSqrZeta(double t_com_position_z, double t_zmp_position_z,
                      double t_com_acceleration_z);
double computeSqrZeta(double t_com_position_z, double t_zmp_position_z,
                      double t_reation_force_z, double t_mass);
double computeSqrZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                      const Vec3D& t_com_acceleration,
                      const Vec3D& t_nu = kVec3DZ);
double computeSqrZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                      const Vec3D& t_reaction_force, double t_mass,
                      const Vec3D& t_nu = kVec3DZ);

// functions to compute zeta
double computeZeta(double t_com_position_z, double t_zmp_position_z,
                   double t_com_acceleration_z);
double computeZeta(double t_com_position_z, double t_zmp_position_z,
                   double t_reation_force_z, double t_mass);
double computeZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                   const Vec3D& t_com_acceleration,
                   const Vec3D& t_nu = kVec3DZ);
double computeZeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                   const Vec3D& t_reaction_force, double t_mass,
                   const Vec3D& t_nu = kVec3DZ);

// functions to compute reaction force
Vec3D computeReactForce(const Vec3D& t_com_acceleration, double t_mass);
Vec3D computeReactForce(const Vec3D& t_com_position,
                        const Vec3D& t_zmp_position, double t_sqr_zeta,
                        double t_mass);
Vec3D computeReactForce(const Vec3D& t_com_position,
                        const Vec3D& t_zmp_position,
                        const Vec3D& t_com_acceleration, double t_mass,
                        const Vec3D& t_nu = kVec3DZ);
Vec3D computeReactForce(const Vec3D& t_com_position,
                        const Vec3D& t_zmp_position, double t_reaction_force_z);

// functions to compute COM acceleration
Vec3D computeComAcc(const Vec3D& t_reaction_force, double t_mass,
                    const Vec3D& t_external_force = kVec3DZero);
Vec3D computeComAcc(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                    double t_sqr_zeta, double t_mass = 1,
                    const Vec3D& t_external_force = kVec3DZero);
Vec3D computeComAcc(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                    const Vec3D& t_reaction_force, double t_mass,
                    const Vec3D& t_external_force = kVec3DZero,
                    const Vec3D& t_nu = kVec3DZ);

// functions to check if some relation is correct
bool isMassValid(double t_mass);
bool isComZmpDiffValid(double t_com_position_z, double t_zmp_position_z);
bool isComZmpDiffValid(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                       const Vec3D& t_nu = kVec3DZ);
bool isReactionForceValid(double t_reaction_force_z);
bool isReactionForceValid(const Vec3D& t_reaction_force,
                          const Vec3D& t_nu = kVec3DZ);
bool isComAccelerationValid(double t_com_acceleration_z);
bool isComAccelerationValid(const Vec3D& t_com_acceleration,
                            const Vec3D& t_nu = kVec3DZ);

}  // namespace ComZmpModelFormula

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
