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

#include "holon/corelib/humanoid/com_ctrl_y.hpp"

#include <zm/zm_misc.h>

namespace holon {

const double ComCtrlY::default_q1 = 1.0;
const double ComCtrlY::default_q2 = 1.0;
const double ComCtrlY::default_rho = 0.0;
const double ComCtrlY::default_dist = 0.0;
const double ComCtrlY::default_kr = 1.0;

ComCtrlY::ComCtrlY(double t_q1, double t_q2, double t_dist)
    : m_q1(t_q1),
      m_q2(t_q2),
      m_rho(default_rho),
      m_dist(t_dist),
      m_kr(default_kr) {}

ComCtrlY::ComCtrlY(double t_q1, double t_q2)
    : ComCtrlY(t_q1, t_q2, default_dist) {}

ComCtrlY::ComCtrlY() : ComCtrlY(default_q1, default_q2, default_dist) {}

ComCtrlY& ComCtrlY::set_q1(double t_q1) {
  m_q1 = t_q1;
  return *this;
}

ComCtrlY& ComCtrlY::set_q2(double t_q2) {
  m_q2 = t_q2;
  return *this;
}

ComCtrlY& ComCtrlY::set_rho(double t_rho) {
  m_rho = t_rho;
  return *this;
}

ComCtrlY& ComCtrlY::set_dist(double t_dist) {
  m_dist = t_dist;
  return *this;
}

ComCtrlY& ComCtrlY::set_kr(double t_kr) {
  m_kr = t_kr;
  return *this;
}

double ComCtrlY::computeNonlinearDumping(double t_yd, double t_y, double t_v,
                                         double t_zeta) const noexcept {
  if (zIsTiny(m_rho) || m_rho < 0.0) return 1.0;
  if (zIsTiny(m_dist) || m_dist < 0.0) return 1.0;
  double r2 = zSqr(t_y - t_yd) + zSqr(t_v / t_zeta) / (m_q1 * m_q2);
  double rz = 0.5 * m_dist;
  return 1.0 - m_rho * exp(m_kr * (1.0 - zSqr((m_q1 * m_q2 + 1.0) / rz) * r2));
}

double ComCtrlY::computeDesZmpPos(double t_yd, double t_y, double t_v,
                                  double t_zeta) const noexcept {
  if (zIsTiny(t_zeta) || t_zeta < 0) {
    ZRUNERROR("ZETA should be positive. (given: %f)", t_zeta);
    return 0;
  }
  double nd = computeNonlinearDumping(t_yd, t_y, t_v, t_zeta);
  return t_y + (m_q1 * m_q2) * (t_y - t_yd) + (m_q1 + m_q2) * nd * t_v / t_zeta;
}

double ComCtrlY::computeDesZmpPos(const Vec3D& t_ref_com_position,
                                  const Vec3D& t_com_position,
                                  const Vec3D& t_com_velocity,
                                  double t_zeta) const noexcept {
  return computeDesZmpPos(t_ref_com_position.y(), t_com_position.y(),
                          t_com_velocity.y(), t_zeta);
}

}  // namespace holon
