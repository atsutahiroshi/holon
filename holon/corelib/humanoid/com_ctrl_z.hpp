/* com_ctrl_z - COM controller along z axis
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

#ifndef HOLON_HUMANOID_COM_CTRL_Z_HPP_
#define HOLON_HUMANOID_COM_CTRL_Z_HPP_

#include "holon/corelib/math/vec3d.hpp"

namespace holon {

class ComCtrlZ {
 public:
  static const double default_q1;
  static const double default_q2;

 public:
  // constructors
  ComCtrlZ();
  ComCtrlZ(double t_q1, double t_q2);

  // special member functions
  virtual ~ComCtrlZ() = default;
  ComCtrlZ(const ComCtrlZ&) = delete;
  ComCtrlZ(ComCtrlZ&&) = delete;
  ComCtrlZ& operator=(const ComCtrlZ&) = delete;
  ComCtrlZ& operator=(ComCtrlZ&&) = delete;

  // accessors
  inline double q1() const noexcept { return m_q1; }
  inline double q2() const noexcept { return m_q2; }

  // mutators
  ComCtrlZ& set_q1(double t_q1);
  ComCtrlZ& set_q2(double t_q2);

  // functions
  double computeDesSqrXi(double t_zd) const;
  double computeDesXi(double t_zd) const;
  double computeDesReactForce(double t_zd, double t_z, double t_v,
                              double t_mass) const noexcept;
  double computeDesReactForce(const Vec3D& t_ref_com_position,
                              const Vec3D& t_com_position,
                              const Vec3D& t_com_velocity, double t_mass) const
      noexcept;

 private:
  double m_q1;
  double m_q2;
};
}

#endif  // HOLON_HUMANOID_COM_CTRL_Z_HPP_
