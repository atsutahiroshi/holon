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
  static constexpr double default_mass = 1;
  double mass = default_mass;
  State position = State{0};
  State velocity = State{0};
  State acceleration = State{0};
  State force = State{0};
  PointMassModelRawData() = default;
  PointMassModelRawData(double t_mass) : mass(t_mass) {}
  PointMassModelRawData(double t_mass, State t_position)
      : mass(t_mass), position(t_position) {}
  PointMassModelRawData(double t_mass, State t_position, State t_velocity,
                        State t_acceleration, State t_force)
      : mass(t_mass),
        position(t_position),
        velocity(t_velocity),
        acceleration(t_acceleration),
        force(t_force) {}
};

template <typename State>
constexpr double PointMassModelRawData<State>::default_mass;

template <typename State>
class PointMassModelData : public DataSetBase<PointMassModelRawData<State>> {
  using Self = PointMassModelData<State>;
  using RawData = PointMassModelRawData<State>;
  using Base = DataSetBase<RawData>;

 public:
  template <typename... Args>
  PointMassModelData(Args... args) : Base(args...) {}

  PointMassModelData(const State& t_initial_position)
      : PointMassModelData(t_initial_position, RawData::default_mass) {}
  PointMassModelData(const State& t_initial_position, double t_mass)
      : PointMassModelData(
            alloc_raw_data<RawData>(t_mass, t_initial_position)) {}
};

}  // namespace holon

#endif  // HOLON_CONTROL_POINT_MASS_MODEL_DATA_HPP_
