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

#include <array>
#include <functional>
#include "holon/corelib/control/point_mass_model/point_mass_model_data.hpp"

namespace holon {

template <typename State, typename Data>
class SystemBase {
 protected:
  using Self = SystemBase<State, Data>;
  using StateArray = std::array<State, 2>;
  using DataPtr = std::shared_ptr<Data>;

 public:
  SystemBase(DataPtr t_data_ptr) : m_data_ptr(t_data_ptr) {}

  // virtual function
  virtual StateArray operator()(const StateArray& state,
                                const double t) const = 0;

  // accessors
  DataPtr data_ptr() const noexcept { return m_data_ptr; }
  Data data() const noexcept { return *m_data_ptr; }

  // mutators
  Self& set_data_ptr(DataPtr t_data_ptr) {
    m_data_ptr = t_data_ptr;
    return *this;
  }

 private:
  DataPtr m_data_ptr;
};

template <typename State, typename Data = PointMassModelData<State>>
class PointMassModelSystem : public SystemBase<State, Data> {
  using Self = PointMassModelSystem<State, Data>;
  using Base = SystemBase<State, Data>;
  using StateArray = typename Base::StateArray;
  using DataPtr = typename Base::DataPtr;

 public:
  using Function =
      std::function<State(const State&, const State&, const double)>;

 public:
  explicit PointMassModelSystem(DataPtr t_data_ptr) : Base(t_data_ptr) {}

  StateArray operator()(const StateArray& state, const double t) const {
    StateArray dxdt;
    dxdt[0] = state[1];
    dxdt[1] = acceleration(state[0], state[1], t);
    return dxdt;
  }

  State acceleration(const State& p, const State& v, const double t) const {
    if (f_acceleration) return f_acceleration(p, v, t);
    return force(p, v, t) / this->data().mass;
  }

  State force(const State& p, const State& v, const double t) const {
    if (!f_force) return State(0.0);
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

 private:
  Function f_acceleration = nullptr;
  Function f_force = nullptr;
};

template <typename State>
PointMassModelSystem<State> makePointMassModelSystem(
    PointMassModelDataPtr<State> t_data_ptr) {
  return PointMassModelSystem<State>(t_data_ptr);
}

}  // namespace holon

#endif  // HOLON_CONTROL_POINT_MASS_MODEL_SYSTEM_HPP_
