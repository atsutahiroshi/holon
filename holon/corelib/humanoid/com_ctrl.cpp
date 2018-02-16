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

#include "holon/corelib/humanoid/com_ctrl.hpp"

namespace holon {

ComCtrl::ComCtrl()
    : m_x(),
      m_y(),
      m_model(),
      m_cmd_com_position(model().com_position()),
      m_des_zmp_position(0, 0, 0) {
  m_des_zeta = computeDesZeta(model().com_position());
}

ComCtrl& ComCtrl::set_time_step(double t_time_step) {
  m_model.set_time_step(t_time_step);
  return *this;
}

ComCtrl& ComCtrl::set_cmd_com_position(const Vec3D& t_cmd_com_position) {
  m_cmd_com_position = t_cmd_com_position;
  return *this;
}

double ComCtrl::computeDesZetaSqr(const Vec3D& t_ref_com_position) const {
  return m_model.computeSqrZeta(t_ref_com_position);
}

double ComCtrl::computeDesZeta(const Vec3D& t_ref_com_position) const {
  return m_model.computeZeta(t_ref_com_position);
}

Vec3D ComCtrl::computeDesZmpPos(const Vec3D& t_ref_com_pos,
                                const Vec3D& t_com_pos, const Vec3D& t_com_vel,
                                double t_desired_zeta) const {
  Vec3D desired_zmp_pos;
  desired_zmp_pos.set_x(m_x.computeDesZmpPos(t_ref_com_pos, t_com_pos,
                                             t_com_vel, t_desired_zeta));
  desired_zmp_pos.set_y(m_y.computeDesZmpPos(t_ref_com_pos, t_com_pos,
                                             t_com_vel, t_desired_zeta));
  desired_zmp_pos.set_z(0);
  return desired_zmp_pos;
}

bool ComCtrl::update() {
  m_des_zeta = computeDesZeta(m_cmd_com_position);
  if (zIsTiny(m_des_zeta)) return false;
  m_des_zmp_position =
      computeDesZmpPos(m_cmd_com_position, m_model.com_position(),
                       m_model.com_velocity(), m_des_zeta);
  m_model.set_zmp_position(m_des_zmp_position);
  return m_model.update();
}

bool ComCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
