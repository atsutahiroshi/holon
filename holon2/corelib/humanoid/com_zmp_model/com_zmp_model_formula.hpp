/* com_zmp_model_formula - Mathematical formulae related to COM-ZMP model
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

#include <cmath>
#include "holon2/corelib/math/vec.hpp"

namespace holon {
namespace com_zmp_model_formula {

// functions to compute zeta squared
double zetaSqr(double t_com_position_z, double t_zmp_position_z,
               double t_com_acceleration_z);
double zetaSqr(double t_com_position_z, double t_zmp_position_z,
               double t_reation_force_z, double t_mass);
double zetaSqr(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               const Vec3d& t_com_acceleration, const Vec3d& t_nu = kVec3dZ);
double zetaSqr(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               const Vec3d& t_contact_force, double t_mass,
               const Vec3d& t_nu = kVec3dZ);

// functions to compute zeta
template <typename... Args>
double zeta(Args&&... args) {
  return std::sqrt(zetaSqr(std::forward<Args>(args)...));
}

// functions to compute contact force
Vec3d contactForce(const Vec3d& t_com_acceleration, double t_mass);
Vec3d contactForce(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                   double t_sqr_zeta, double t_mass);
Vec3d contactForce(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                   const Vec3d& t_com_acceleration, double t_mass,
                   const Vec3d& t_nu = kVec3dZ);
Vec3d contactForce(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                   double t_contact_force_z);

// functions to compute COM acceleration
Vec3d comAccel(const Vec3d& t_contact_force, double t_mass,
               const Vec3d& t_external_force = kVec3dZero);
Vec3d comAccel(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               double t_sqr_zeta, double t_mass = 1,
               const Vec3d& t_external_force = kVec3dZero);
Vec3d comAccel(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               const Vec3d& t_contact_force, double t_mass,
               const Vec3d& t_external_force = kVec3dZero,
               const Vec3d& t_nu = kVec3dZ);

}  // namespace com_zmp_model_formula
}  // namespace holon
