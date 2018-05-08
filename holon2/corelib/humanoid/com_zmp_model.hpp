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

#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_builder.hpp"
#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"
#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_simulator.hpp"

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
  Vec3d reaction_force;
  Vec3d external_force;
  Vec3d total_force;
};
using ComZmpModelData = Dataset<ComZmpModelParams, ComZmpModelStates>;

class ComZmpModel {
  using Params = ComZmpModelParams;
  using States = ComZmpModelStates;
  using Data = ComZmpModelData;

 public:
  ComZmpModel() : m_data() {}
  explicit ComZmpModel(Data t_data) : m_data(t_data) {}

  // special member functions
  ComZmpModel(const ComZmpModel&) = delete;
  ComZmpModel& operator=(const ComZmpModel&) = delete;
  ComZmpModel(ComZmpModel&&) = default;
  ComZmpModel& operator=(ComZmpModel&&) = default;
  virtual ~ComZmpModel() = default;

  const Data& data() const noexcept { return m_data; }
  const Params& params() const noexcept { return m_data.get<0>(); }
  const States& states() const noexcept { return m_data.get<1>(); }

  Data& data() {
    return const_cast<Data&>(static_cast<const ComZmpModel&>(*this).data());
  }
  Params& params() {
    return const_cast<Params&>(static_cast<const ComZmpModel&>(*this).params());
  }
  States& states() {
    return const_cast<States&>(static_cast<const ComZmpModel&>(*this).states());
  }

  double mass() const noexcept { return params().mass; }
  Vec3d com_position() const noexcept { return states().com_position; }
  Vec3d com_velocity() const noexcept { return states().com_velocity; }
  Vec3d com_acceleration() const noexcept { return states().com_acceleration; }
  Vec3d zmp_position() const noexcept { return states().zmp_position; }
  Vec3d reaction_force() const noexcept { return states().reaction_force; }
  Vec3d external_force() const noexcept { return states().external_force; }
  Vec3d total_force() const noexcept { return states().total_force; }

  ComZmpModel clone() const { return ComZmpModel(data().clone()); }

 private:
  Data m_data;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_HPP_
