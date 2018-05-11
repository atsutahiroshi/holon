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

#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"
#include "holon2/corelib/humanoid/const_defs.hpp"
#include "holon2/corelib/math/misc.hpp"

namespace holon {
namespace com_zmp_model_formula {

namespace {

bool isMassValid(double t_mass) {
  if (isTiny(t_mass) || t_mass < 0.0) {
    // ZRUNWARN("The mass must be positive. (given mass = %g)", t_mass);
    return false;
  }
  return true;
}

bool isComZmpDiffValid(double t_com_position_z, double t_zmp_position_z) {
  double diff = t_com_position_z - t_zmp_position_z;
  if (isTiny(diff) || diff < 0.0) {
    // ZRUNWARN("The COM must be above the terrain. (given z = %g, zz = %g)",
    //          t_com_position_z, t_zmp_position_z);
    return false;
  }
  return true;
}

bool isComZmpDiffValid(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                       const Vec3d& t_nu = kVec3dZ) {
  double com_side = t_nu.dot(t_com_position - t_zmp_position);
  if (isTiny(com_side) || com_side < 0.0) {
    // ZRUNWARN(
    //     "The must be above the terrain. "
    //     "(COM: %s, ZMP: %s, nu: %s)",
    //     t_com_position.str().c_str(), t_zmp_position.str().c_str(),
    //     t_nu.str().c_str());
    return false;
  }
  return true;
}

bool isContactForceValid(double t_contact_force_z) {
  if (t_contact_force_z < 0.0) {
    // ZRUNWARN("The contact force must be positive. (given %g)",
    //          t_contact_force_z);
    return false;
  }
  return true;
}

bool isContactForceValid(const Vec3d& t_contact_force,
                         const Vec3d& t_nu = kVec3dZ) {
  double contact_force = t_nu.dot(t_contact_force);
  if (contact_force < 0.0) {
    // ZRUNWARN(
    //     "The contact force must be positive. "
    //     "(force: %s, nu: %s)",
    //     t_contact_force.str().c_str(), t_nu.str().c_str());
    return false;
  }
  return true;
}

bool isComAccelValid(double t_com_acceleration_z) {
  if ((t_com_acceleration_z + kGravAccel) < 0.0) {
    // ZRUNWARN("The COM acceleration must be greater than -G. (given %g)",
    //          t_com_acceleration_z);
    return false;
  }
  return true;
}

bool isComAccelValid(const Vec3d& t_com_acceleration,
                     const Vec3d& t_nu = kVec3dZ) {
  double acc = t_nu.dot(t_com_acceleration + kGravAccel3d);
  if (acc < 0.0) {
    // ZRUNWARN("Cannot produce pulling force (COM acc: %s, nu: %s)",
    //          t_com_acceleration.str().c_str(), t_nu.str().c_str());
    return false;
  }
  return true;
}

}  // namespace

double zetaSqr(double t_com_position_z, double t_zmp_position_z,
               double t_com_acceleration_z) {
  if (!isComZmpDiffValid(t_com_position_z, t_zmp_position_z) ||
      !isComAccelValid(t_com_acceleration_z)) {
    return 0.0;
  }
  double numer = t_com_acceleration_z + kGravAccel;
  double denom = t_com_position_z - t_zmp_position_z;
  return numer / denom;
}

double zetaSqr(double t_com_position_z, double t_zmp_position_z,
               double t_contact_force_z, double t_mass) {
  if (!isMassValid(t_mass) ||
      !isComZmpDiffValid(t_com_position_z, t_zmp_position_z) ||
      !isContactForceValid(t_contact_force_z)) {
    return 0.0;
  }
  double denom = (t_com_position_z - t_zmp_position_z) * t_mass;
  return t_contact_force_z / denom;
}

double zetaSqr(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               const Vec3d& t_com_acceleration, const Vec3d& t_nu) {
  if (!isComZmpDiffValid(t_com_position, t_zmp_position) ||
      !isComAccelValid(t_com_acceleration)) {
    return 0.0;
  }
  double numer = t_nu.dot(t_com_acceleration + kGravAccel3d);
  double denom = t_nu.dot(t_com_position - t_zmp_position);
  return numer / denom;
}

double zetaSqr(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               const Vec3d& t_contact_force, double t_mass, const Vec3d& t_nu) {
  if (!isMassValid(t_mass) ||
      !isComZmpDiffValid(t_com_position, t_zmp_position) ||
      !isContactForceValid(t_contact_force)) {
    return 0.0;
  }
  double numer = t_nu.dot(t_contact_force);
  double denom = t_nu.dot(t_com_position - t_zmp_position) * t_mass;
  return numer / denom;
}

Vec3d contactForce(const Vec3d& t_com_acceleration, double t_mass) {
  return t_mass * (t_com_acceleration + kGravAccel3d);
}

Vec3d contactForce(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                   double t_zeta_sqr, double t_mass) {
  return t_mass * t_zeta_sqr * (t_com_position - t_zmp_position);
}

Vec3d contactForce(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                   const Vec3d& t_com_acceleration, double t_mass,
                   const Vec3d& t_nu) {
  double zeta_sqr =
      zetaSqr(t_com_position, t_zmp_position, t_com_acceleration, t_nu);
  return contactForce(t_com_position, t_zmp_position, zeta_sqr, t_mass);
}

Vec3d contactForce(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
                   double t_contact_force_z) {
  if (!isComZmpDiffValid(t_com_position, t_zmp_position))
    return Vec3d(0, 0, t_contact_force_z);
  double m_zeta_sqr =
      t_contact_force_z / (t_com_position.z() - t_zmp_position.z());
  return m_zeta_sqr * (t_com_position - t_zmp_position);
}

Vec3d comAccel(const Vec3d& t_contact_force, double t_mass,
               const Vec3d& t_external_force) {
  return (t_contact_force + t_external_force) / t_mass - kGravAccel3d;
}

Vec3d comAccel(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               double t_zeta_sqr, double t_mass,
               const Vec3d& t_external_force) {
  return t_zeta_sqr * (t_com_position - t_zmp_position) - kGravAccel3d +
         t_external_force / t_mass;
}

Vec3d comAccel(const Vec3d& t_com_position, const Vec3d& t_zmp_position,
               const Vec3d& t_contact_force, double t_mass,
               const Vec3d& t_external_force, const Vec3d& t_nu) {
  double zeta_sqr =
      zetaSqr(t_com_position, t_zmp_position, t_contact_force, t_mass, t_nu);
  return comAccel(t_com_position, t_zmp_position, zeta_sqr, t_mass,
                  t_external_force);
}

}  // namespace com_zmp_model_formula
}  // namespace holon
