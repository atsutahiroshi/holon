/* contact_force_generator - Contact force generator class
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

#include "holon2/corelib/humanoid/com_controller/contact_force_generator.hpp"

#include "holon2/corelib/humanoid/const_defs.hpp"
#include "holon2/corelib/math/misc.hpp"

namespace holon {

ContactForceGenerator::ContactForceGenerator(const Data& t_data)
    : m_data(t_data) {}

ContactForceGenerator& ContactForceGenerator::setData(const Data& t_data) {
  m_data = t_data;
  return *this;
}

namespace {

double calculateXiSqr(double t_zd) {
  if (isTiny(t_zd) || t_zd < 0) {
    // ZRUNERROR("Desired COM height should be positive. (given: %f)", t_zd);
    return 0;
  }
  return kGravAccel / t_zd;
}

}  // namespace

double ContactForceGenerator::calculateZ(const double t_z,
                                         const double t_v) const {
  double xi2 = calculateXiSqr(params().com_position[2]);
  double xi = sqrt(xi2);
  double z = t_z - params().com_position[2];
  double k1 = params().q1[2] * params().q2[2];
  double k2 = params().q1[2] + params().q2[2];
  double fz = -xi2 * k1 * z - xi * k2 * t_v + kGravAccel;
  fz *= mass();
  return std::max<double>(fz, 0);
}

Vec3d ContactForceGenerator::calculate(const Vec3d& t_com_position,
                                       const Vec3d& t_com_velocity) const {
  auto fz = calculateZ(t_com_position.z(), t_com_velocity.z());
  return Vec3d(0, 0, fz);
}

}  // namespace holon
