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
  using Self = PointMassModel<State, StateArray, Solver, Data, System>;
  using Base = ModelBase<State, Solver, Data, System>;
  using RawData = PointMassModelRawData<State>;

 public:
  using Function = typename System::Function;

 public:
  PointMassModel() : PointMassModel(State{0}, RawData::default_mass) {}
  explicit PointMassModel(const State& t_initial_position)
      : PointMassModel(t_initial_position, RawData::default_mass) {}
  PointMassModel(const State& t_initial_position, double t_mass)
      : PointMassModel(make_data<Data>(t_initial_position, t_mass)) {}
  explicit PointMassModel(Data t_data)
      : Base(t_data), m_initial_position(Base::states().position) {}
  virtual ~PointMassModel() = default;

  // accessors
  double mass() const noexcept { return this->states().mass; }
  State initial_position() const noexcept { return m_initial_position; }

  // mutators
  Self& set_initial_position(const State& t_initial_position) {
    m_initial_position = t_initial_position;
    return *this;
  }
  virtual Self& reset() override {
    Base::reset();
    this->states().position = m_initial_position;
    this->states().velocity = State{0};
    return *this;
  }
  virtual Self& reset(const State& t_initial_position) {
    this->set_initial_position(t_initial_position);
    return this->reset();
  }

  // callback functions
  Self& setForceCallback(Function t_force_f) {
    this->system().set_force(t_force_f);
    return *this;
  }
  Self& setAccelerationCallback(Function t_acceleration_f) {
    this->system().set_acceleration(t_acceleration_f);
    return *this;
  }

  // update
  virtual bool update() override {
    auto p = this->states().position;
    auto v = this->states().velocity;
    update_data(p, v);
    if (!Base::update()) return false;
    return true;
  }

  virtual bool update(double dt) override {
    this->set_time_step(dt);
    return update();
  }

 private:
  State m_initial_position;

  void update_data(const State& p, const State& v) {
    StateArray state{p, v};
    state = this->solver().update(this->system(), state, this->time(),
                                  this->time_step());
    this->states().force = this->system().force(p, v, this->time());
    this->states().acceleration =
        this->system().acceleration(p, v, this->time());
    this->states().position = state[0];
    this->states().velocity = state[1];
  }
};

}  // namespace holon

#endif  // HOLON_CONTROL_POINT_MASS_MODEL_HPP_
