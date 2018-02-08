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

#include "holon/corelib/humanoid/com_ctrl_x.hpp"
#include "holon/corelib/humanoid/com_ctrl_y.hpp"
#include "holon/corelib/humanoid/com_zmp_model.hpp"

namespace holon {

class ComCtrl {
 public:
  ComCtrl();
  virtual ~ComCtrl();

  inline const ComCtrlX& x() const noexcept { return m_x; }
  inline ComCtrlX& x() noexcept { return m_x; }

  inline const ComCtrlY& y() const noexcept { return m_y; }
  inline ComCtrlY& y() noexcept { return m_y; }

  inline const ComZmpModel& model() const noexcept { return m_model; }

  inline const zVec3D* cmd_com_position() const noexcept {
    return &m_cmd_com_position;
  }
  inline zVec3D* cmd_com_position() noexcept { return &m_cmd_com_position; }
  ComCtrl& set_cmd_com_position(const zVec3D* t_cmd_com_position);

  inline const zVec3D* des_zmp_position() const noexcept {
    return &m_des_zmp_position;
  }
  inline zVec3D* des_zmp_position() noexcept { return &m_des_zmp_position; }

  double computeDesiredZetaSqr(const zVec3D* ref_com_position) const;
  double computeDesiredZeta(const zVec3D* ref_com_position) const;

  zVec3D* computeDesiredZmpPosition(const zVec3D* ref_com_position,
                                    const zVec3D* com_position,
                                    const zVec3D* com_velocity,
                                    zVec3D* desired_zmp_position) const;

 private:
  ComCtrlX m_x;
  ComCtrlY m_y;
  ComZmpModel m_model;

  zVec3D m_cmd_com_position;
  zVec3D m_des_zmp_position;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_HPP_
