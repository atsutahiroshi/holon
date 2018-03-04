/* com_ctrl_z - COM controller along z axis
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

#include "holon/corelib/humanoid/com_ctrl_z.hpp"

#include <roki/rk_g.h>
#include <algorithm>

namespace holon {
namespace com_ctrl_z {

namespace {

double computeSqrXi(double t_zd) {
  if (zIsTiny(t_zd) || t_zd < 0) {
    ZRUNERROR("Desired COM height should be positive. (given: %f)", t_zd);
    return 0;
  }
  return RK_G / t_zd;
}

}  // namespace

double computeDesReactForce(double t_z, double t_v, double t_zd, double t_q1,
                            double t_q2, double t_mass) {
  double xi2 = computeSqrXi(t_zd);
  double xi = sqrt(xi2);
  double fz =
      -xi2 * t_q1 * t_q2 * (t_z - t_zd) - xi * (t_q1 + t_q2) * t_v + RK_G;
  fz *= t_mass;
  return std::max<double>(fz, 0);
}

double computeDesReactForce(const Vec3D& t_com_position,
                            const Vec3D& t_com_velocity,
                            const Vec3D& t_ref_com_position, double t_q1,
                            double t_q2, double t_mass) {
  return computeDesReactForce(t_com_position.z(), t_com_velocity.z(),
                              t_ref_com_position.z(), t_q1, t_q2, t_mass);
}
double computeDesReactForce(const Vec3D& t_com_position,
                            const Vec3D& t_com_velocity,
                            const Parameters& t_parameters) {
  return computeDesReactForce(t_com_position.z(), t_com_velocity.z(),
                              t_parameters.zd, t_parameters.q1, t_parameters.q2,
                              t_parameters.mass);
}

}  // namespace com_ctrl_z
}  // namespace holon
