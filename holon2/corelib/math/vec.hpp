/* vec - Vector class
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
#ifndef HOLON_MATH_VEC_HPP_
#define HOLON_MATH_VEC_HPP_

#include "third_party/eigen/Eigen/Dense"

namespace holon {

using Vec = Eigen::VectorXd;
using Vec3d = Eigen::Vector3d;

}  // namespace holon

#endif  // HOLON_MATH_VEC_HPP_
