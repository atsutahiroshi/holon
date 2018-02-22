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

const double ComCtrlZ::default_q1 = 1.0;
const double ComCtrlZ::default_q2 = 1.0;

ComCtrlZ::ComCtrlZ() : m_q1(default_q1), m_q2(default_q2) {}
ComCtrlZ::ComCtrlZ(double t_q1, double t_q2) : m_q1(t_q1), m_q2(t_q2) {}

ComCtrlZ& ComCtrlZ::set_q1(double t_q1) {
  m_q1 = t_q1;
  return *this;
}

ComCtrlZ& ComCtrlZ::set_q2(double t_q2) {
  m_q2 = t_q2;
  return *this;
}

double ComCtrlZ::computeDesSqrXi(double t_zd) const {
  if (zIsTiny(t_zd) || t_zd < 0) {
    ZRUNERROR("Desired COM height should be positive. (given: %f)", t_zd);
    return 0;
  }
  return RK_G / t_zd;
}

double ComCtrlZ::computeDesXi(double t_zd) const {
  return sqrt(computeDesSqrXi(t_zd));
}

double ComCtrlZ::computeDesReactForce(double t_zd, double t_z, double t_v,
                                      double t_mass) const noexcept {
  double xi2 = computeDesSqrXi(t_zd);
  double xi = sqrt(xi2);
  double fz =
      -xi2 * m_q1 * m_q2 * (t_z - t_zd) - xi * (m_q1 + m_q2) * t_v + RK_G;
  fz *= t_mass;
  return std::max<double>(fz, 0);
}

double ComCtrlZ::computeDesReactForce(const Vec3D& t_ref_com_position,
                                      const Vec3D& t_com_position,
                                      const Vec3D& t_com_velocity,
                                      double t_mass) const noexcept {
  return computeDesReactForce(t_ref_com_position.z(), t_com_position.z(),
                              t_com_velocity.z(), t_mass);
}

}  // namespace holon
