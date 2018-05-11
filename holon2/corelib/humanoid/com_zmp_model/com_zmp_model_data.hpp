/* com_zmp_model_data - Data for COM-ZMP model
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

#ifndef HOLON_HUMANOID_COM_ZMP_MODEL_DATA_HPP_
#define HOLON_HUMANOID_COM_ZMP_MODEL_DATA_HPP_

#include "holon2/corelib/dataset/dataset.hpp"
#include "holon2/corelib/math/vec.hpp"

namespace holon {

struct ComZmpModelParams {
  double mass;
  Vec3d nu;
  double vhp;
};
struct ComZmpModelStates {
  Vec3d com_position;
  Vec3d com_velocity;
  Vec3d com_acceleration;
  Vec3d zmp_position;
  Vec3d contact_force;
  Vec3d external_force;
  Vec3d reaction_force;
};
using ComZmpModelData = Dataset<ComZmpModelParams, ComZmpModelStates>;

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_DATA_HPP_