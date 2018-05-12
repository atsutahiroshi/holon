/* biped_foot_model - Biped foot model
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

#ifndef HOLON_HUMANOID_BIPED_FOOT_MODEL_HPP_
#define HOLON_HUMANOID_BIPED_FOOT_MODEL_HPP_

#include "holon2/corelib/humanoid/biped_foot_model/biped_foot_model_builder.hpp"
#include "holon2/corelib/humanoid/biped_foot_model/biped_foot_model_data.hpp"

namespace holon {

class BipedFootModel {
  using Params = BipedFootModelParams;
  using States = BipedFootModelStates;
  using Data = BipedFootModelData;
  using Self = BipedFootModel;

 public:
  BipedFootModel() : m_data() {}
  explicit BipedFootModel(const Data& t_data) : m_data(t_data) {}

  // special member functions
  BipedFootModel(const Self&) = delete;
  Self& operator=(const Self&) = delete;
  BipedFootModel(Self&&) = default;
  Self& operator=(Self&&) = default;

  const Data& data() const noexcept { return m_data; }
  const Params& params() const noexcept { return m_data.get<0>(); }
  const States& states() const noexcept { return m_data.get<1>(); }

  Data& data() {
    return const_cast<Data&>(static_cast<const Self&>(*this).data());
  }
  Params& params() {
    return const_cast<Params&>(static_cast<const Self&>(*this).params());
  }
  States& states() {
    return const_cast<States&>(static_cast<const Self&>(*this).states());
  }

  double mass() const noexcept { return params().mass; }
  Vec3d position() const noexcept { return states().position; }
  Vec3d velocity() const noexcept { return states().velocity; }
  Vec3d acceleration() const noexcept { return states().acceleration; }
  Vec3d force() const noexcept { return states().force; }

  Self clone() const { return Self(data().clone()); }

 private:
  Data m_data;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_FOOT_MODEL_HPP_
