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

#include "holon/corelib/humanoid/com_ctrl_x.hpp"

#include <zm/zm_misc.h>

namespace holon {

ComCtrlX::ComCtrlX() : m_q1(default_q1), m_q2(default_q2) {}
ComCtrlX::ComCtrlX(double t_q1, double t_q2) : m_q1(t_q1), m_q2(t_q2) {}

ComCtrlX::~ComCtrlX() = default;

ComCtrlX& ComCtrlX::set_q1(double t_q1) {
  m_q1 = t_q1;
  return *this;
}

ComCtrlX& ComCtrlX::set_q2(double t_q2) {
  m_q2 = t_q2;
  return *this;
}

double ComCtrlX::computeDesiredZmpPosition(double xd, double x, double v,
                                           double zeta) const noexcept {
  if (zIsTiny(zeta) || zeta < 0) {
    ZRUNERROR("ZETA should be positive. (given: %f)", zeta);
    return 0;
  }
  return x + (m_q1 * m_q2) * (x - xd) + (m_q1 + m_q2) * v / zeta;
}

double ComCtrlX::computeDesiredZmpPosition(const zVec3D& ref_com_position,
                                           const zVec3D& com_position,
                                           const zVec3D& com_veocity,
                                           double zeta) const noexcept {
  return computeDesiredZmpPosition(zVec3DElem(&ref_com_position, zX),
                                   zVec3DElem(&com_position, zX),
                                   zVec3DElem(&com_veocity, zX), zeta);
}

}  // namespace holon
