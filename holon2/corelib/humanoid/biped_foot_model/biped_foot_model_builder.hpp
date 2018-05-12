/* biped_foot_model_builder - Builder class of biped foot model
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

#ifndef HOLON_HUMANOID_BIPED_FOOT_MODEL_BIPED_FOOT_MODEL_BUILDER_HPP_
#define HOLON_HUMANOID_BIPED_FOOT_MODEL_BIPED_FOOT_MODEL_BUILDER_HPP_

#include "holon2/corelib/dataset/dataset.hpp"
#include "holon2/corelib/math/vec.hpp"

namespace holon {

struct BipedFootModelParams;
struct BipedFootModelStates;
using BipedFootModelData = Dataset<BipedFootModelParams, BipedFootModelStates>;
class BipedFootModel;

class BipedFootModelBuilder {
  using Self = BipedFootModelBuilder;
  using Params = BipedFootModelParams;
  using States = BipedFootModelStates;
  using Data = BipedFootModelData;
  using Model = BipedFootModel;

 public:
  static const double default_mass;
  static const Vec3d default_position;

 public:
  BipedFootModelBuilder();

  Self& setMass(const double t_mass);
  Self& setPosition(const Vec3d& t_position);
  Self& setVelocity(const Vec3d& t_velocity);

  Model build();
  Model build(Data t_data);

 private:
  double m_mass;
  Vec3d m_position;
  Vec3d m_velocity;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_FOOT_MODEL_BIPED_FOOT_MODEL_BUILDER_HPP_
