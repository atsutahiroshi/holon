/* zmp_manipulator - ZMP manipulator class
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

#ifndef HOLON_HUMANOID_COM_CONTROLLER_ZMP_MANIPULATOR_HPP_
#define HOLON_HUMANOID_COM_CONTROLLER_ZMP_MANIPULATOR_HPP_

#include "holon2/corelib/humanoid/com_controller/com_controller_data.hpp"

namespace holon {

class ZmpManipulator {
  using Self = ZmpManipulator;
  using Data = ComControllerData;
  using Params = ComControllerParams;

 public:
  explicit ZmpManipulator(const Data& t_data);

  const Data& data() const { return m_data; }
  const Params& params() const { return m_data.get<2>(); }
  double vhp() const { return m_data.get<0>().vhp; }
  double mass() const { return m_data.get<0>().mass; }
  Self& setData(const Data& t_data);

  double calculateX(const double t_x, const double t_v,
                    const double t_zeta) const;
  double calculateY(const double t_y, const double t_v,
                    const double t_zeta) const;
  Vec3d calculate(const Vec3d& t_com_position, const Vec3d& t_com_velocity,
                  const double t_zeta) const;

 private:
  Data m_data;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CONTROLLER_ZMP_MANIPULATOR_HPP_
