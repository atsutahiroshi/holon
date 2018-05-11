/* zmp_manipulator - ZMP manipulator class
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

#include "holon2/corelib/humanoid/com_controller/zmp_manipulator.hpp"

#include "holon2/corelib/math/misc.hpp"

namespace holon {

ZmpManipulator::ZmpManipulator(const Data& t_data) : m_data(t_data) {}

ZmpManipulator& ZmpManipulator::setData(const Data& t_data) {
  m_data = t_data;
  return *this;
}

double ZmpManipulator::calculateX(const double t_com_position_x,
                                  const double t_com_velocity_y,
                                  const double t_zeta) {
  if (isTiny(t_zeta) || t_zeta < 0) {
    // ZRUNERROR("ZETA should be positive. (given: %f)", t_zeta);
    return 0;
  }
  // double x = t_x - t_xd;
  // double v = t_v - t_vd;
  // return t_x + (t_q1 * t_q2) * x + (t_q1 + t_q2) * v / t_zeta;
}

}  // namespace holon
