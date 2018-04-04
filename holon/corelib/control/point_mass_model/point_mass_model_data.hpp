/* point_mass_model_data - Data for point mass model
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

#ifndef HOLON_CONTROL_POINT_MASS_MODEL_DATA_HPP_
#define HOLON_CONTROL_POINT_MASS_MODEL_DATA_HPP_

#include <memory>
#include "holon/corelib/data/data_set_base.hpp"

namespace holon {

template <typename State>
struct PointMassModelRawData {
  double mass;
  State position;
  State velocity;
  State acceleration;
  State force;
};

template <typename State>
class PointMassModelData : public DataSetBase<PointMassModelRawData<State>> {
  using Self = PointMassModelData<State>;
  using RawData = PointMassModelRawData<State>;
  using Base = DataSetBase<RawData>;

 public:
  static constexpr double default_mass = 1;

 public:
  PointMassModelData(const RawData& t_raw_data) : Base(t_raw_data) {}
  PointMassModelData(std::shared_ptr<RawData> t_raw_data_p)
      : Base(t_raw_data_p) {}
  PointMassModelData() : PointMassModelData(State{0}, default_mass) {}
  PointMassModelData(const State& t_initial_position)
      : PointMassModelData(t_initial_position, default_mass) {}
  PointMassModelData(const State& t_initial_position, double t_mass)
      : PointMassModelData(std::move(RawData{t_mass, t_initial_position,
                                             State{0}, State{0}, State{0}})) {}
};

template <typename State>
constexpr double PointMassModelData<State>::default_mass;

}  // namespace holon

#endif  // HOLON_CONTROL_POINT_MASS_MODEL_DATA_HPP_
