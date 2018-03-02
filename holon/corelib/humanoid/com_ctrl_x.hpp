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

#ifndef HOLON_HUMANOID_COM_CTRL_X_HPP_
#define HOLON_HUMANOID_COM_CTRL_X_HPP_

#include "holon/corelib/math/vec3d.hpp"

namespace holon {

class ComCtrlX {
 public:
  static const double default_q1;
  static const double default_q2;
  static const double default_vd;

 public:
  // constructors
  ComCtrlX();
  ComCtrlX(double t_q1, double t_q2);

  // special member functions
  virtual ~ComCtrlX() = default;
  ComCtrlX(const ComCtrlX&) = delete;
  ComCtrlX(ComCtrlX&&) = delete;
  ComCtrlX& operator=(const ComCtrlX&) = delete;
  ComCtrlX& operator=(ComCtrlX&&) = delete;

  // accessors
  inline double q1() const noexcept { return m_q1; }
  inline double q2() const noexcept { return m_q2; }
  inline double vd() const noexcept { return m_vd; }

  // mutators
  ComCtrlX& set_q1(double t_q1);
  ComCtrlX& set_q2(double t_q2);
  ComCtrlX& set_vd(double t_vd);

  // functions
  double computeDesZmpPos(double t_xd, double t_x, double t_v,
                          double t_zeta) const noexcept;
  double computeDesZmpPos(const Vec3D& t_ref_com_position,
                          const Vec3D& t_com_position,
                          const Vec3D& t_com_velocity, double t_zeta) const
      noexcept;

 private:
  double m_q1;
  double m_q2;
  double m_vd;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_X_HPP_
