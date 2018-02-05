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

#include "holon/corelib/humanoid/com_zmp_model.hpp"

namespace holon {

class ComCtrl {
  const double default_q1 = 1.0;
  const double default_q2 = 1.0;

 public:
  ComCtrl();
  ComCtrl(double t_q1, double t_q2);
  virtual ~ComCtrl();

  inline double q1() const noexcept { return m_q1; };
  inline double q2() const noexcept { return m_q2; };

  ComCtrl& set_q1(double t_q1);
  ComCtrl& set_q2(double t_q2);

  const ComZmpModel& model() const noexcept { return m_model; };

  double ComputeDesiredZetaSqr(const zVec3D* ref_com_position) const;
  double ComputeDesiredZeta(const zVec3D* ref_com_position) const;

  zVec3D* ComputeDesiredZmpPosition(const zVec3D* ref_com_position,
                                    const zVec3D* com_position,
                                    const zVec3D* com_velocity,
                                    zVec3D* desired_zmp_position) const;

 private:
  ComZmpModel m_model;
  double m_q1, m_q2;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CTRL_HPP_