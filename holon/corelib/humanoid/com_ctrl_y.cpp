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

ComCtrlY::ComCtrlY() : m_q1(default_q1), m_q2(default_q2) {}
ComCtrlY::ComCtrlY(double t_q1, double t_q2) : m_q1(t_q1), m_q2(t_q2) {}

ComCtrlY& ComCtrlY::set_q1(double t_q1) {
  m_q1 = t_q1;
  return *this;
}

ComCtrlY& ComCtrlY::set_q2(double t_q2) {
  m_q2 = t_q2;
  return *this;
}

double ComCtrlY::computeDesiredZmpPosition(double yd, double y, double v,
                                           double zeta) const noexcept {
  if (zIsTiny(zeta) || zeta < 0) {
    ZRUNERROR("ZETA should be positive. (given: %f)", zeta);
    return 0;
  }
  return y + (m_q1 * m_q2) * (y - yd) + (m_q1 + m_q2) * v / zeta;
}

double ComCtrlY::computeDesiredZmpPosition(const zVec3D& ref_com_position,
                                           const zVec3D& com_position,
                                           const zVec3D& com_veocity,
                                           double zeta) const noexcept {
  return computeDesiredZmpPosition(zVec3DElem(&ref_com_position, zY),
                                   zVec3DElem(&com_position, zY),
                                   zVec3DElem(&com_veocity, zY), zeta);
}

}  // namespace holon
