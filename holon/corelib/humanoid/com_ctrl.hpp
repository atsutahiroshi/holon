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

#include "holon/corelib/humanoid/com_ctrl_x.hpp"
#include "holon/corelib/humanoid/com_ctrl_y.hpp"
#include "holon/corelib/humanoid/com_zmp_model.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

class ComCtrl {
 public:
  // constructors
  ComCtrl();

  // special member functions
  virtual ~ComCtrl() = default;
  ComCtrl(const ComCtrl&) = delete;
  ComCtrl(ComCtrl&&) = delete;
  ComCtrl& operator=(const ComCtrl&) = delete;
  ComCtrl& operator=(ComCtrl&&) = delete;

  // accessors
  inline const ComCtrlX& x() const noexcept { return m_x; }
  inline ComCtrlX& x() noexcept { return m_x; }
  inline const ComCtrlY& y() const noexcept { return m_y; }
  inline ComCtrlY& y() noexcept { return m_y; }
  inline const ComZmpModel& model() const noexcept { return m_model; }
  inline ComZmpModel& model() noexcept { return m_model; }
  inline double time_step() const noexcept { return model().time_step(); }
  inline const Vec3D cmd_com_position() const noexcept {
    return m_cmd_com_position;
  }
  inline const Vec3D des_zmp_position() const noexcept {
    return m_des_zmp_position;
  }
  inline double des_zeta() const noexcept { return m_des_zeta; }

  // mutators
  ComCtrl& set_time_step(double t_time_step);
  ComCtrl& set_cmd_com_position(const Vec3D& t_cmd_com_position);

  // computing functions
  double computeDesZetaSqr(const Vec3D& t_ref_com_position) const;
  double computeDesZeta(const Vec3D& t_ref_com_position) const;
  Vec3D computeDesZmpPos(const Vec3D& t_ref_com_position,
                         const Vec3D& t_com_position,
                         const Vec3D& t_com_velocity,
                         double t_desired_zeta) const;

  // update functions
  bool update();
  bool update(double t_time_step);

 private:
  ComCtrlX m_x;
  ComCtrlY m_y;
  ComZmpModel m_model;

  Vec3D m_cmd_com_position;
  Vec3D m_des_zmp_position;
  double m_des_zeta;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_HPP_
