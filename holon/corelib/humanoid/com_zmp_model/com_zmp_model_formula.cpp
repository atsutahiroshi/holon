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

#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

#include <roki/rk_g.h>

namespace holon {
namespace com_zmp_model_formula {

namespace {
const Vec3D kG = {0, 0, RK_G};
}  // namespace

double squared_zeta(double t_com_position_z, double t_zmp_position_z,
                    double t_com_acceleration_z) {
  if (!is_com_zmp_diff_valid(t_com_position_z, t_zmp_position_z) ||
      !is_com_acceleration_valid(t_com_acceleration_z)) {
    return 0.0;
  }
  double numer = t_com_acceleration_z + RK_G;
  double denom = t_com_position_z - t_zmp_position_z;
  return numer / denom;
}

double squared_zeta(double t_com_position_z, double t_zmp_position_z,
                    double t_reaction_force_z, double t_mass) {
  if (!is_mass_valid(t_mass) ||
      !is_com_zmp_diff_valid(t_com_position_z, t_zmp_position_z) ||
      !is_reaction_force_valid(t_reaction_force_z)) {
    return 0.0;
  }
  double denom = (t_com_position_z - t_zmp_position_z) * t_mass;
  return t_reaction_force_z / denom;
}

double squared_zeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                    const Vec3D& t_com_acceleration, const Vec3D& t_nu) {
  if (!is_com_zmp_diff_valid(t_com_position, t_zmp_position) ||
      !is_com_acceleration_valid(t_com_acceleration)) {
    return 0.0;
  }
  double numer = t_nu.dot(t_com_acceleration + kG);
  double denom = t_nu.dot(t_com_position - t_zmp_position);
  return numer / denom;
}

double squared_zeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                    const Vec3D& t_reaction_force, double t_mass,
                    const Vec3D& t_nu) {
  if (!is_mass_valid(t_mass) ||
      !is_com_zmp_diff_valid(t_com_position, t_zmp_position) ||
      !is_reaction_force_valid(t_reaction_force)) {
    return 0.0;
  }
  double numer = t_nu.dot(t_reaction_force);
  double denom = t_nu.dot(t_com_position - t_zmp_position) * t_mass;
  return numer / denom;
}

double zeta(double t_com_position_z, double t_zmp_position_z,
            double t_com_acceleration_z) {
  return sqrt(
      squared_zeta(t_com_position_z, t_zmp_position_z, t_com_acceleration_z));
}

double zeta(double t_com_position_z, double t_zmp_position_z,
            double t_reation_force_z, double t_mass) {
  return sqrt(squared_zeta(t_com_position_z, t_zmp_position_z,
                           t_reation_force_z, t_mass));
}

double zeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
            const Vec3D& t_com_acceleration, const Vec3D& t_nu) {
  return sqrt(
      squared_zeta(t_com_position, t_zmp_position, t_com_acceleration, t_nu));
}

double zeta(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
            const Vec3D& t_reaction_force, double t_mass, const Vec3D& t_nu) {
  return sqrt(squared_zeta(t_com_position, t_zmp_position, t_reaction_force,
                           t_mass, t_nu));
}

Vec3D reaction_force(const Vec3D& t_com_acceleration, double t_mass) {
  return t_mass * (t_com_acceleration + kG);
}

Vec3D reaction_force(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                     double t_sqr_zeta, double t_mass) {
  return t_mass * t_sqr_zeta * (t_com_position - t_zmp_position);
}

Vec3D reaction_force(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                     const Vec3D& t_com_acceleration, double t_mass,
                     const Vec3D& t_nu) {
  double sqr_zeta =
      squared_zeta(t_com_position, t_zmp_position, t_com_acceleration, t_nu);
  return reaction_force(t_com_position, t_zmp_position, sqr_zeta, t_mass);
}

Vec3D reaction_force(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                     double t_reaction_force_z) {
  if (!is_com_zmp_diff_valid(t_com_position, t_zmp_position))
    return Vec3D(0, 0, t_reaction_force_z);
  double m_sqr_zeta =
      t_reaction_force_z / (t_com_position.z() - t_zmp_position.z());
  return m_sqr_zeta * (t_com_position - t_zmp_position);
}

Vec3D com_acceleration(const Vec3D& t_reaction_force, double t_mass,
                       const Vec3D& t_external_force) {
  return (t_reaction_force + t_external_force) / t_mass - kG;
}

Vec3D com_acceleration(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                       double t_sqr_zeta, double t_mass,
                       const Vec3D& t_external_force) {
  return t_sqr_zeta * (t_com_position - t_zmp_position) - kG +
         t_external_force / t_mass;
}
Vec3D com_acceleration(const Vec3D& t_com_position, const Vec3D& t_zmp_position,
                       const Vec3D& t_reaction_force, double t_mass,
                       const Vec3D& t_external_force, const Vec3D& t_nu) {
  double sqr_zeta = squared_zeta(t_com_position, t_zmp_position,
                                 t_reaction_force, t_mass, t_nu);
  return com_acceleration(t_com_position, t_zmp_position, sqr_zeta, t_mass,
                          t_external_force);
}

bool is_mass_valid(double t_mass) {
  if (zIsTiny(t_mass) || t_mass < 0.0) {
    ZRUNWARN("The mass must be positive. (given mass = %g)", t_mass);
    return false;
  }
  return true;
}

bool is_com_zmp_diff_valid(double t_com_position_z, double t_zmp_position_z) {
  double diff = t_com_position_z - t_zmp_position_z;
  if (zIsTiny(diff) || diff < 0.0) {
    ZRUNWARN("The COM must be above the terrain. (given z = %g, zz = %g)",
             t_com_position_z, t_zmp_position_z);
    return false;
  }
  return true;
}

bool is_com_zmp_diff_valid(const Vec3D& t_com_position,
                           const Vec3D& t_zmp_position, const Vec3D& t_nu) {
  double com_side = t_nu.dot(t_com_position - t_zmp_position);
  if (zIsTiny(com_side) || com_side < 0.0) {
    ZRUNWARN(
        "The must be above the terrain. "
        "(COM: %s, ZMP: %s, nu: %s)",
        t_com_position.str().c_str(), t_zmp_position.str().c_str(),
        t_nu.str().c_str());
    return false;
  }
  return true;
}

bool is_reaction_force_valid(double t_reaction_force_z) {
  if (t_reaction_force_z < 0.0) {
    ZRUNWARN("The reaction force must be positive. (given %g)",
             t_reaction_force_z);
    return false;
  }
  return true;
}

bool is_reaction_force_valid(const Vec3D& t_reaction_force, const Vec3D& t_nu) {
  double react_force = t_nu.dot(t_reaction_force);
  if (react_force < 0.0) {
    ZRUNWARN(
        "The reaction force must be positive. "
        "(force: %s, nu: %s)",
        t_reaction_force.str().c_str(), t_nu.str().c_str());
    return false;
  }
  return true;
}

bool is_com_acceleration_valid(double t_com_acceleration_z) {
  if ((t_com_acceleration_z + RK_G) < 0.0) {
    ZRUNWARN("The COM acceleration must be greater than -G. (given %g)",
             t_com_acceleration_z);
    return false;
  }
  return true;
}

bool is_com_acceleration_valid(const Vec3D& t_com_acceleration,
                               const Vec3D& t_nu) {
  double acc = t_nu.dot(t_com_acceleration + kG);
  if (acc < 0.0) {
    ZRUNWARN("Cannot produce pulling force (COM acc: %s, nu: %s)",
             t_com_acceleration.str().c_str(), t_nu.str().c_str());
    return false;
  }
  return true;
}

}  // namespace com_zmp_model_formula
}  // namespace holon
