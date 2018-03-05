/* com_ctrl_y - COM controller along y axis
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

#include "holon/corelib/humanoid/com_ctrl/com_ctrl_y.hpp"

#include <zm/zm_misc.h>

namespace holon {
namespace com_ctrl_y {

namespace {

double computeNonlinearDumping(double t_y, double t_v, double t_yd, double t_q1,
                               double t_q2, double t_rho, double t_dist,
                               double t_kr, double t_zeta) {
  if (zIsTiny(t_rho) || t_rho < 0.0) return 1.0;
  if (zIsTiny(t_dist) || t_dist < 0.0) return 1.0;
  double r2 = zSqr(t_y - t_yd) + zSqr(t_v / t_zeta) / (t_q1 * t_q2);
  double rz = 0.5 * t_dist;
  return 1.0 - t_rho * exp(t_kr * (1.0 - zSqr((t_q1 * t_q2 + 1.0) / rz) * r2));
}

}  // namespace

double computeDesZmpPos(double t_y, double t_v, double t_yd, double t_q1,
                        double t_q2, double t_rho, double t_dist, double t_kr,
                        double t_zeta) {
  if (zIsTiny(t_zeta) || t_zeta < 0) {
    ZRUNERROR("ZETA should be positive. (given: %f)", t_zeta);
    return 0;
  }
  double nd = computeNonlinearDumping(t_y, t_v, t_yd, t_q1, t_q2, t_rho, t_dist,
                                      t_kr, t_zeta);
  return t_y + (t_q1 * t_q2) * (t_y - t_yd) + (t_q1 + t_q2) * nd * t_v / t_zeta;
}

double computeDesZmpPos(const Vec3D& t_com_position,
                        const Vec3D& t_com_velocity,
                        const Vec3D& t_ref_com_position, double t_q1,
                        double t_q2, double t_rho, double t_dist, double t_kr,
                        double t_zeta) {
  return computeDesZmpPos(t_com_position.y(), t_com_velocity.y(),
                          t_ref_com_position.y(), t_q1, t_q2, t_rho, t_dist,
                          t_kr, t_zeta);
}

double computeDesZmpPos(const Vec3D& t_com_position,
                        const Vec3D& t_com_velocity,
                        const Parameters& t_parameters) {
  return computeDesZmpPos(t_com_position.y(), t_com_velocity.y(),
                          t_parameters.yd, t_parameters.q1, t_parameters.q2,
                          t_parameters.rho, t_parameters.dist, t_parameters.kr,
                          t_parameters.zeta);
}

}  // namespace com_ctrl_y
}  // namespace holon
