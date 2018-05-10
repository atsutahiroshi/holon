/* com_zmp_model_builder - Builder class of COM-ZMP model
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

#ifndef HOLON_HUMANOID_COM_ZMP_MODEL_BUILDER_HPP_
#define HOLON_HUMANOID_COM_ZMP_MODEL_BUILDER_HPP_

#include "holon2/corelib/dataset/dataset.hpp"
#include "holon2/corelib/math/vec.hpp"

namespace holon {

struct ComZmpModelParams;
struct ComZmpModelStates;
using ComZmpModelData = Dataset<ComZmpModelParams, ComZmpModelStates>;
class ComZmpModel;

class ComZmpModelBuilder {
  using Self = ComZmpModelBuilder;
  using Params = ComZmpModelParams;
  using States = ComZmpModelStates;
  using Data = ComZmpModelData;
  using Model = ComZmpModel;

 public:
  static const double default_mass;
  static const double default_vhp;
  static const Vec3d default_com_position;

 public:
  ComZmpModelBuilder();

  Self& setMass(const double t_mass);
  Self& setVirtualHorizontalPlane(const double t_vhp);
  Self& setComPosition(const Vec3d& t_com_position);
  Self& setComVelocity(const Vec3d& t_com_velocity);

  Model build();
  Model build(Data t_data);

 private:
  double m_mass;
  double m_vhp;
  Vec3d m_com_position;
  Vec3d m_com_velocity;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_BUILDER_HPP_
