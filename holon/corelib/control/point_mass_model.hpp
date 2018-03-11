/* point_mass_model - Point mass model
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

#ifndef HOLON_CONTROL_POINT_MASS_MODEL_HPP_
#define HOLON_CONTROL_POINT_MASS_MODEL_HPP_

#include <memory>
#include "holon/corelib/control/model_base.hpp"
#include "holon/corelib/control/point_mass_model/point_mass_model_data.hpp"
#include "holon/corelib/control/point_mass_model/point_mass_model_system.hpp"
#include "holon/corelib/math/ode_runge_kutta4.hpp"

namespace holon {

template <typename State, typename StateArray = std::array<State, 2>,
          typename Solver = RungeKutta4<StateArray>,
          typename Data = PointMassModelData<State>,
          typename System = PointMassModelSystem<State, Data>>
class PointMassModel : public ModelBase<State, Solver, Data, System> {
  using Self = PointMassModel;
  using Base = ModelBase<State, Solver, Data, System>;
  using DataPtr = std::shared_ptr<Data>;
  using Function = typename System::Function;

 public:
  PointMassModel();
  explicit PointMassModel(const State& t_initial_poisition);
  PointMassModel(const State& t_initial_poisition, double t_mass);
  explicit PointMassModel(DataPtr t_data_ptr);

  // accessors
  double mass() const noexcept { return this->data().mass; }
  State initial_position() const noexcept { return m_initial_position; }

  // mutators
  Self& set_initial_position(const State& t_initial_position);
  virtual Self& reset() override;
  virtual Self& reset(const State& t_initial_position);

  // callback functions
  Self& setForceCallback(Function t_force_f);
  Self& setAccelerationCallback(Function t_acceleration_f);

  // update
  virtual bool update() override;
  virtual bool update(double dt) override;

 private:
  State m_initial_position;
};

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
PointMassModel<State, StateArray, Solver, Data, System>::PointMassModel()
    : PointMassModel(State(0), Data::default_mass) {}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
PointMassModel<State, StateArray, Solver, Data, System>::PointMassModel(
    const State& t_initial_position)
    : PointMassModel(t_initial_position, Data::default_mass) {}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
PointMassModel<State, StateArray, Solver, Data, System>::PointMassModel(
    const State& t_initial_position, double t_mass)
    : PointMassModel(createPointMassModelData(t_initial_position, t_mass)) {}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
PointMassModel<State, StateArray, Solver, Data, System>::PointMassModel(
    DataPtr t_data_ptr)
    : ModelBase<State, Solver, Data, System>(t_data_ptr),
      m_initial_position(Base::data().position) {}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
PointMassModel<State, StateArray, Solver, Data, System>&
PointMassModel<State, StateArray, Solver, Data, System>::set_initial_position(
    const State& t_initial_position) {
  m_initial_position = t_initial_position;
  return *this;
}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
PointMassModel<State, StateArray, Solver, Data, System>&
PointMassModel<State, StateArray, Solver, Data, System>::reset() {
  Base::reset();
  this->data_ptr()->position = m_initial_position;
  this->data_ptr()->velocity = State{0};
  return *this;
}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
PointMassModel<State, StateArray, Solver, Data, System>&
PointMassModel<State, StateArray, Solver, Data, System>::reset(
    const State& t_initial_position) {
  set_initial_position(t_initial_position);
  return reset();
}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
PointMassModel<State, StateArray, Solver, Data, System>&
PointMassModel<State, StateArray, Solver, Data, System>::setForceCallback(
    Function t_force_f) {
  this->system().set_force(t_force_f);
  return *this;
}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
PointMassModel<State, StateArray, Solver, Data, System>&
PointMassModel<State, StateArray, Solver, Data,
               System>::setAccelerationCallback(Function t_acceleration_f) {
  this->system().set_acceleration(t_acceleration_f);
  return *this;
}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
bool PointMassModel<State, StateArray, Solver, Data, System>::update() {
  StateArray state{{this->data().position, this->data().velocity}};
  state = this->solver().update(this->system(), state, this->time(),
                                this->time_step());
  this->data_ptr()->position = state[0];
  this->data_ptr()->velocity = state[1];
  if (!Base::update()) return false;
  return true;
}

template <typename State, typename StateArray, typename Solver, typename Data,
          typename System>
bool PointMassModel<State, StateArray, Solver, Data, System>::update(
    double dt) {
  this->set_time_step(dt);
  return update();
}

// non-member functions
template <typename State>
PointMassModel<State> makePointMassModel() {
  return PointMassModel<State>();
}

template <typename State>
PointMassModel<State> makePointMassModel(const State& t_initial_position) {
  return PointMassModel<State>(t_initial_position);
}

template <typename State>
PointMassModel<State> makePointMassModel(const State& t_initial_position,
                                         double t_mass) {
  return PointMassModel<State>(t_initial_position, t_mass);
}

}  // namespace holon

#endif  // HOLON_CONTROL_POINT_MASS_MODEL_HPP_
