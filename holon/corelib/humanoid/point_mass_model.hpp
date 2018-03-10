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

#ifndef HOLON_HUMANOID_POINT_MASS_MODEL_HPP_
#define HOLON_HUMANOID_POINT_MASS_MODEL_HPP_

#include <memory>
#include "holon/corelib/humanoid/point_mass_model/point_mass_model_data.hpp"
#include "holon/corelib/humanoid/point_mass_model/point_mass_model_system.hpp"
#include "holon/corelib/math/ode_euler.hpp"

namespace holon {

template <typename State, typename Solver, typename Data, typename System>
class ModelBase {
 protected:
  using Self = ModelBase;
  using DataPtr = std::shared_ptr<Data>;
  using Function = typename System::Function;

 public:
  static constexpr double default_time_step = 0.001;

 public:
  explicit ModelBase(DataPtr t_data_ptr)
      : m_time(0.0),
        m_time_step(default_time_step),
        m_data_ptr(t_data_ptr),
        m_system(t_data_ptr),
        m_solver() {}

  // accessors
  double time() const noexcept { return m_time; }
  double time_step() const noexcept { return m_time_step; }
  Data data() const noexcept { return *m_data_ptr; }
  DataPtr data_ptr() const noexcept { return m_data_ptr; }
  System& system() noexcept { return m_system; }
  const System& system() const noexcept { return m_system; }
  Solver solver() noexcept { return m_solver; }
  const Solver& solver() const noexcept { return m_solver; }

  // mutators
  virtual Self& set_time_step(double t_time_step) {
    m_time_step = t_time_step;
    return *this;
  }
  virtual Self& set_data_ptr(DataPtr t_data_ptr) {
    m_data_ptr = t_data_ptr;
    return *this;
  }
  virtual Self& reset() {
    m_time = 0;
    return *this;
  }

  virtual bool update() {
    m_time += m_time_step;
    return true;
  }

  virtual bool update(double dt) {
    set_time_step(dt);
    return update();
  }

 private:
  double m_time;
  double m_time_step;
  DataPtr m_data_ptr;
  System m_system;
  Solver m_solver;
};

template <typename State, typename Solver, typename Data, typename System>
constexpr double ModelBase<State, Solver, Data, System>::default_time_step;

template <typename State, typename StateArray = std::array<State, 2>,
          typename Solver = Euler<StateArray>,
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

#endif  // HOLON_HUMANOID_POINT_MASS_MODEL_HPP_
