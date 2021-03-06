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

#ifndef HOLON_HUMANOID_COM_CTRL_X_HPP_
#define HOLON_HUMANOID_COM_CTRL_X_HPP_

#include "holon/corelib/math/vec3d.hpp"

namespace holon {
namespace com_ctrl_x {

struct Parameters {
  double xd;
  double vd;
  double q1, q2;
  double zeta;
};

static const double default_q1 = 1;
static const double default_q2 = 1;

double computeDesZmpPos(double t_x, double t_v, double t_xd, double t_vd,
                        double t_q1, double t_q2, double t_zeta);
double computeDesZmpPos(const Vec3D& t_com_position,
                        const Vec3D& t_com_velocity,
                        const Vec3D& t_ref_com_position,
                        const Vec3D& t_ref_com_velocity, double t_q1,
                        double t_q2, double t_zeta);
double computeDesZmpPos(const Vec3D& t_com_position,
                        const Vec3D& t_com_velocity,
                        const Parameters& t_parameters);

}  // namespace com_ctrl_x
}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_X_HPP_
