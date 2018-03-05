/* com_ctrl_x - COM controller along x axis
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

#include "holon/corelib/humanoid/com_ctrl/com_ctrl_x.hpp"

#include <zm/zm_misc.h>

namespace holon {
namespace com_ctrl_x {

double computeDesZmpPos(double t_x, double t_v, double t_xd, double t_vd,
                        double t_q1, double t_q2, double t_zeta) {
  if (zIsTiny(t_zeta) || t_zeta < 0) {
    ZRUNERROR("ZETA should be positive. (given: %f)", t_zeta);
    return 0;
  }
  double x = t_x - t_xd;
  double v = t_v - t_vd;
  return t_x + (t_q1 * t_q2) * x + (t_q1 + t_q2) * v / t_zeta;
}

double computeDesZmpPos(const Vec3D& t_com_position,
                        const Vec3D& t_com_velocity,
                        const Vec3D& t_ref_com_position,
                        const Vec3D& t_ref_com_velocity, double t_q1,
                        double t_q2, double t_zeta) {
  return computeDesZmpPos(t_com_position.x(), t_com_velocity.x(),
                          t_ref_com_position.x(), t_ref_com_velocity.x(), t_q1,
                          t_q2, t_zeta);
}

double computeDesZmpPos(const Vec3D& t_com_position,
                        const Vec3D& t_com_velocity,
                        const Parameters& t_parameters) {
  return computeDesZmpPos(t_com_position.x(), t_com_velocity.x(),
                          t_parameters.xd, t_parameters.vd, t_parameters.q1,
                          t_parameters.q2, t_parameters.zeta);
}

}  // namespace com_ctrl_x
}  // namespace holon
