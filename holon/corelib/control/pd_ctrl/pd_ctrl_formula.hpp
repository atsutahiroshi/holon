/* pd_ctrl_formula - Helper functions for simple PD control class
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

#ifndef HOLON_CONTROL_PD_CTRL_FORMULA_HPP_
#define HOLON_CONTROL_PD_CTRL_FORMULA_HPP_

#include "holon/corelib/math/vec3d.hpp"

namespace holon {
namespace pd_ctrl_formula {

inline double computeDesForce(const double& t_x, const double& t_v,
                              const double& t_xd, const double& t_vd,
                              const double& t_k, const double& t_c) {
  return t_k * (t_xd - t_x) + t_c * (t_vd - t_v);
}

inline Vec3D computeDesForce(const Vec3D& t_x, const Vec3D& t_v,
                             const Vec3D& t_xd, const Vec3D& t_vd,
                             const Vec3D& t_k, const Vec3D& t_c) {
  Vec3D res;
  for (auto i = 0; i < 3; ++i) {
    res[i] = computeDesForce(t_x[i], t_v[i], t_xd[i], t_vd[i], t_k[i], t_c[i]);
  }
  return res;
}

}  // namespace pd_ctrl_formula
}  // namespace holon

#endif  // HOLON_CONTROL_PD_CTRL_FORMULA_HPP_
