/* point_mass_model_system - System definition of point mass model
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

#ifndef HOLON_CONTROL_POINT_MASS_MODEL_SYSTEM_HPP_
#define HOLON_CONTROL_POINT_MASS_MODEL_SYSTEM_HPP_

#include <functional>
#include "holon/corelib/control/point_mass_model/point_mass_model_data.hpp"
#include "holon/corelib/control/system_base.hpp"

namespace holon {

template <typename State, typename Data = PointMassModelData<State>>
class PointMassModelSystem : public SystemBase<State, Data> {
  using Self = PointMassModelSystem<State, Data>;
  using Base = SystemBase<State, Data>;
  using StateArray = typename Base::StateArray;

 public:
  using Function =
      std::function<State(const State&, const State&, const double)>;

 public:
  explicit PointMassModelSystem(Data t_data) : Base(t_data) {}

  virtual StateArray operator()(const StateArray& state,
                                const double t) const override {
    StateArray dxdt;
    dxdt[0] = state[1];
    dxdt[1] = acceleration(state[0], state[1], t);
    return dxdt;
  }

  State acceleration(const State& p, const State& v, const double t) const {
    if (f_acceleration) return f_acceleration(p, v, t);
    return force(p, v, t) / this->data().template get<0>().mass;
  }

  State force(const State& p, const State& v, const double t) const {
    if (!f_force) return State{0.0};
    return f_force(p, v, t);
  }

  Self& set_acceleration(Function t_acceleration) {
    f_acceleration = t_acceleration;
    return *this;
  }

  Self& set_force(Function t_force) {
    f_force = t_force;
    return *this;
  }

  Self& setConstForce(const State& t_force) {
    return set_force([t_force](const State&, const State&, const double) {
      return t_force;
    });
  }

  bool is_set_force() { return f_force != nullptr; }
  bool is_set_acceleration() { return f_acceleration != nullptr; }

 private:
  Function f_acceleration = nullptr;
  Function f_force = nullptr;
};

}  // namespace holon

#endif  // HOLON_CONTROL_POINT_MASS_MODEL_SYSTEM_HPP_
