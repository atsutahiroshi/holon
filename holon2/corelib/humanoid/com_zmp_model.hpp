/* com_zmp_model - COM-ZMP model
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of holon.
 *
 * Holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
#define HOLON_HUMANOID_COM_ZMP_MODEL_HPP_

#include <cmath>
#include <utility>
#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_builder.hpp"

namespace holon {

struct ComZmpModelParams {
  double mass;
  Vec3d nu;
  double vhp;
};
struct ComZmpModelStates {
  Vec3d com_position;
  Vec3d com_velocity;
  Vec3d com_acceleration;
  Vec3d zmp_position;
  Vec3d reaction_force;
  Vec3d external_force;
  Vec3d total_force;
};
using ComZmpModelData = Dataset<ComZmpModelParams, ComZmpModelStates>;

class ComZmpModel {
  using Params = ComZmpModelParams;
  using States = ComZmpModelStates;
  using Data = ComZmpModelData;

 public:
  ComZmpModel() : m_data() {}
  explicit ComZmpModel(Data t_data) : m_data(t_data) {}

  const Data& data() const { return m_data; }
  const Params& params() const { return m_data.get<0>(); }
  const States& states() const { return m_data.get<1>(); }

 private:
  Data m_data;
};

namespace com_zmp_model_formula {

// functions to compute zeta squared
double zetaSqr(double t_com_position_z, double t_zmp_position_z,
               double t_com_acceleration_z);
double zetaSqr(double t_com_position_z, double t_zmp_position_z,
               double t_reation_force_z, double t_mass);
double zetaSqr(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               const Vec3d& t_com_acceleration, const Vec3d& t_nu = kVec3dZ);
double zetaSqr(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               const Vec3d& t_reaction_force, double t_mass,
               const Vec3d& t_nu = kVec3dZ);

// functions to compute zeta
template <typename... Args>
double zeta(Args&&... args) {
  return std::sqrt(zetaSqr(std::forward<Args>(args)...));
}

// functions to compute reaction force
Vec3d reactForce(const Vec3d& t_com_acceleration, double t_mass);
Vec3d reactForce(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                 double t_sqr_zeta, double t_mass);
Vec3d reactForce(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                 const Vec3d& t_com_acceleration, double t_mass,
                 const Vec3d& t_nu = kVec3dZ);
Vec3d reactForce(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                 double t_reaction_force_z);

// functions to compute COM acceleration
Vec3d comAccel(const Vec3d& t_reaction_force, double t_mass,
               const Vec3d& t_external_force = kVec3dZero);
Vec3d comAccel(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               double t_sqr_zeta, double t_mass = 1,
               const Vec3d& t_external_force = kVec3dZero);
Vec3d comAccel(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               const Vec3d& t_reaction_force, double t_mass,
               const Vec3d& t_external_force = kVec3dZero,
               const Vec3d& t_nu = kVec3dZ);

}  // namespace com_zmp_model_formula

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
