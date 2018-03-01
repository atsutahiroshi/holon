/* com_zmp_model_formula - Formulae related to COM-ZMP model
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

#ifndef HOLON_HUMANOID_COM_ZMP_MODEL_FORMULA_HPP_
#define HOLON_HUMANOID_COM_ZMP_MODEL_FORMULA_HPP_

#include "holon/corelib/math/vec3d.hpp"

namespace holon {
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

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_FORMULA_HPP_
