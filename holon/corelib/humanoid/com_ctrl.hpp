/* com_ctrl - COM Controller
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

#ifndef HOLON_HUMANOID_COM_CTRL_HPP_
#define HOLON_HUMANOID_COM_CTRL_HPP_

#include <zeo/zeo_vec3d.h>

namespace holon {

class ComCtrl {
 public:
  ComCtrl();
  virtual ~ComCtrl();

  zVec3D* ComputeDesiredZmpPosition(const zVec3D* ref_com_position,
                                    const zVec3D* com_position,
                                    const zVec3D* com_velocity,
                                    zVec3D* desired_zmp_position) const;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_HPP_
