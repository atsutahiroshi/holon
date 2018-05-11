/* contact_force_generator - Contact force generator class
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

#ifndef HOLON_HUMANOID_COM_CONTROLLER_CONTACT_FORCE_GENERATOR_HPP_
#define HOLON_HUMANOID_COM_CONTROLLER_CONTACT_FORCE_GENERATOR_HPP_

#include "holon2/corelib/humanoid/com_controller/com_controller_data.hpp"

namespace holon {

class ContactForceGenerator {
  using Self = ContactForceGenerator;
  using Data = ComControllerData;
  using Params = ComControllerParams;

 public:
  explicit ContactForceGenerator(const Data& t_data);

  const Data& data() const { return m_data; }
  const Params& params() const { return m_data.get<2>(); }
  double vhp() const { return m_data.get<0>().vhp; }
  double mass() const { return m_data.get<0>().mass; }
  Self& setData(const Data& t_data);

  double calculateZ(const double t_z, const double t_v);
  Vec3d calculate(const Vec3d& t_com_position, const Vec3d& t_com_velocity);

 private:
  Data m_data;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CONTROLLER_CONTACT_FORCE_GENERATOR_HPP_
