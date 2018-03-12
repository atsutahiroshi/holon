/* model_base - Base class for model
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

#ifndef HOLON_CONTROL_MODEL_BASE_HPP_
#define HOLON_CONTROL_MODEL_BASE_HPP_

#include <memory>

namespace holon {

template <typename State, typename Solver, typename Data, typename System>
class ModelBase {
 protected:
  using Self = ModelBase<State, Solver, Data, System>;
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
  const Data& data() const noexcept { return *m_data_ptr; }
  DataPtr data_ptr() const noexcept { return m_data_ptr; }
  System& system() noexcept { return m_system; }
  const System& system() const noexcept { return m_system; }
  Solver& solver() noexcept { return m_solver; }
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

}  // namespace holon

#endif  // HOLON_CONTROL_MODEL_BASE_HPP_
