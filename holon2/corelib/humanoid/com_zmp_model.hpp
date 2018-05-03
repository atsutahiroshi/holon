/* com_zmp_model - COM-ZMP model
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

#ifndef HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
#define HOLON_HUMANOID_COM_ZMP_MODEL_HPP_

#include "holon2/corelib/dataset/dataset.hpp"
#include "holon2/corelib/humanoid/const_defs.hpp"
#include "holon2/corelib/math/vec.hpp"

namespace holon {

struct ComZmpModelParams {
  double mass;
  Vec3d nu;
  double vhp;
};
struct ComZmpModelInputs {
  Vec3d com_position;
  Vec3d com_velocity;
  Vec3d zmp_position;
  Vec3d reaction_force;
  Vec3d external_force;
};
struct ComZmpModelOutputs {
  Vec3d com_position;
  Vec3d com_velocity;
  Vec3d com_acceleration;
  Vec3d zmp_position;
  Vec3d reaction_force;
  Vec3d total_force;
};
using ComZmpModelData =
    Dataset<ComZmpModelParams, ComZmpModelInputs, ComZmpModelOutputs>;

class ComZmpModel;

class ComZmpModelBuilder {
  static const double default_mass;
  static const double default_vhp;
  static const Vec3d default_com_position;
  using Self = ComZmpModelBuilder;
  using Params = ComZmpModelParams;
  using Inputs = ComZmpModelInputs;
  using Outputs = ComZmpModelOutputs;
  using Data = ComZmpModelData;
  using Model = ComZmpModel;

 public:
  ComZmpModelBuilder();

  Self& setMass(const double t_mass);
  Self& setVirtualHorizontalPlane(const double t_vhp);
  Self& setComPosition(const Vec3d& t_com_position);

  Model build();
  Model build(Data t_data);

 private:
  double m_mass;
  double m_vhp;
  Vec3d m_com_position;
};

class ComZmpModel {
  using Params = ComZmpModelParams;
  using Inputs = ComZmpModelInputs;
  using Outputs = ComZmpModelOutputs;
  using Data = ComZmpModelData;

 public:
  ComZmpModel() : m_data() {}
  explicit ComZmpModel(Data t_data) : m_data(t_data) {}

  const Data& data() const { return m_data; }
  const Params& params() const { return m_data.get<0>(); }
  const Inputs& inputs() const { return m_data.get<1>(); }
  const Outputs& outputs() const { return m_data.get<2>(); }

 private:
  Data m_data;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
