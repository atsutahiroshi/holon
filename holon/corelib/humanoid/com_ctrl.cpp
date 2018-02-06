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

class ComCtrl::impl {
  double m_q1, m_q2;

 public:
  impl() : m_q1(1), m_q2(1) {}
  impl(double t_q1, double t_q2) : m_q1(t_q1), m_q2(t_q2) {}
  double compute(double xd, double x, double vx, double zeta) {
    return x + (m_q1 * m_q2) * (x - xd) + (m_q1 + m_q2) * vx / zeta;
  }
  inline double q1() const { return m_q1; }
  inline double q2() const { return m_q2; }
  void set_q1(double t_q1) { m_q1 = t_q1; }
  void set_q2(double t_q2) { m_q2 = t_q2; }
};

ComCtrl::ComCtrl() : m_model(), m_impl(new impl()) {}
ComCtrl::ComCtrl(double t_q1, double t_q2)
    : m_model(), m_impl(new impl(t_q1, t_q2)) {}

ComCtrl::~ComCtrl() = default;

double ComCtrl::q1() const noexcept { return m_impl->q1(); }
double ComCtrl::q2() const noexcept { return m_impl->q2(); }

ComCtrl& ComCtrl::set_q1(double t_q1) {
  m_impl->set_q1(t_q1);
  return *this;
}

ComCtrl& ComCtrl::set_q2(double t_q2) {
  m_impl->set_q2(t_q2);
  return *this;
}

double ComCtrl::ComputeDesiredZetaSqr(const zVec3D* ref_com_position) const {
  return m_model.ComputeZetaSqr(ref_com_position);
}

double ComCtrl::ComputeDesiredZeta(const zVec3D* ref_com_position) const {
  return m_model.ComputeZeta(ref_com_position);
}

zVec3D* ComCtrl::ComputeDesiredZmpPosition(const zVec3D* ref_com_position,
                                           const zVec3D* com_position,
                                           const zVec3D* com_velocity,
                                           zVec3D* desired_zmp_position) const {
  double xd = zVec3DElem(ref_com_position, zX);
  double x = zVec3DElem(com_position, zX);
  double vx = zVec3DElem(com_velocity, zX);
  double yd = zVec3DElem(ref_com_position, zY);
  double y = zVec3DElem(com_position, zY);
  double vy = zVec3DElem(com_velocity, zY);
  double zeta = ComputeDesiredZeta(ref_com_position);
  zVec3DCreate(desired_zmp_position, m_impl->compute(xd, x, vx, zeta),
               m_impl->compute(yd, y, vy, zeta), 0);
  return desired_zmp_position;
}

}  // namespace holon
