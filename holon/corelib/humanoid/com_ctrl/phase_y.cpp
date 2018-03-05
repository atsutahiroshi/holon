/* phase_y - computation of phase along y-axis
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

#include "holon/corelib/humanoid/com_ctrl/phase_y.hpp"

#include <zm/zm_misc.h>

namespace holon {
namespace phase_y {

double computeFrequency(double t_q1, double t_q2, double t_zeta) {
  if (t_zeta < 0 || (t_q1 * t_q2) < 0) {
    ZRUNERROR("ZETA, Q1 and Q2 must be positive. (given: %f, %f, %f)", t_zeta,
              t_q1, t_q2);
    return 0;
  }
  return t_zeta * sqrt(t_q1 * t_q2);
}

}  // namespace phase_y
}  // namespace holon
