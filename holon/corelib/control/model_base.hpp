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

#include <limits>
#include <memory>
#include <type_traits>
#include <utility>
#include "holon/corelib/control/system_base.hpp"
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/math/misc.hpp"
#include "holon/corelib/math/ode_solver.hpp"

namespace holon {

template <typename State, typename Data>
struct DummySystem : SystemBase<State, Data> {
  using Base = SystemBase<State, Data>;
  explicit DummySystem(Data t_data) : Base(t_data) {}
  typename Base::StateArray operator()(const typename Base::StateArray&,
                                       const double) const {
    return typename Base::StateArray{State{0}, State{0}};
  }
};

template <typename State, typename Solver, typename Data,
          typename System = DummySystem<State, Data>>
class ModelBase {
  static_assert(std::is_base_of<OdeSolver<Solver>, Solver>::value,
                "Solver must be derived from OdeSolver class.");
  static_assert(is_data_type<Data>::value,
                "Data must be derived from DataSetBase class.");
  static_assert(std::is_base_of<SystemBase<State, Data>, System>::value,
                "System must be derived from SystemBase class.");

  using Self = ModelBase<State, Solver, Data, System>;

 protected:
  using Function = typename System::Function;

 public:
  static constexpr bool is_model_type = true;
  static constexpr double default_time_step = 0.001;
  using DataType = Data;
  template <std::size_t I>
  using RawDataType = typename Data::template RawDataI<I>;

 public:
  explicit ModelBase(Data t_data)
      : m_time(0.0),
        m_time_step(default_time_step),
        m_data(t_data),
        m_system(t_data),
        m_solver() {}
  virtual ~ModelBase() = default;

  // accessors
  double time() const noexcept { return m_time; }
  double time_step() const noexcept { return m_time_step; }

  const Data& data() const noexcept { return m_data; }
  Data& data() noexcept { return m_data; }

  template <std::size_t I = 0>
  const RawDataType<I>& states() const noexcept {
    return m_data.template get<I>();
  }
  template <std::size_t I = 0>
  RawDataType<I>& states() noexcept {
    return m_data.template get<I>();
  }

  const System& system() const noexcept { return m_system; }
  System& system() noexcept { return m_system; }

  const Solver& solver() const noexcept { return m_solver; }
  Solver& solver() noexcept { return m_solver; }

  // mutators
  Self& set_time_step(double t_time_step) {
    m_time_step = is_positive(t_time_step) ? t_time_step : default_time_step;
    return *this;
  }
  Self& set_data(const Data& t_data) {
    m_data = t_data;
    return *this;
  }
  virtual Self& reset() {
    m_time = 0;
    return *this;
  }

  // copy data
  Self& copy_data(const Data& t_data) { m_data.copy(t_data); }
  Self& copy_data(const Self& t_model) { this->copy_data(t_model.data()); }

  // update
  virtual bool update() {
    m_time += m_time_step;
    return true;
  }
  virtual bool update(double t_time_step) {
    set_time_step(t_time_step);
    return update();
  }

 private:
  double m_time;
  double m_time_step;
  Data m_data;
  System m_system;
  Solver m_solver;
};

template <typename State, typename Solver, typename Data, typename System>
constexpr double ModelBase<State, Solver, Data, System>::default_time_step;

template <typename T, typename = int>
struct is_model_type : std::false_type {};

template <typename T>
struct is_model_type<T, decltype((void)T::is_model_type, 0)> : std::true_type {
};

// non-member functions
template <typename Model, typename... Args>
Model make_model(Args&&... args) {
  static_assert(is_model_type<Model>::value,
                "make_model is able to make type derived from ModelBase only.");
  return Model(std::forward<Args>(args)...);
}

}  // namespace holon

#endif  // HOLON_CONTROL_MODEL_BASE_HPP_
