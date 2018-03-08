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

namespace holon {

template <typename State, typename Data, typename System>
class ModelBase {
 protected:
  using Self = ModelBase;
  using DataPtr = std::shared_ptr<Data>;
  using Function = typename System::Function;

 public:
  static constexpr double default_time_step = 0.001;

 public:
  explicit ModelBase(DataPtr t_data_ptr)
      : m_time(0.0), m_time_step(default_time_step), m_data_ptr(t_data_ptr) {}

  // accessors
  double time() const noexcept { return m_time; }
  double time_step() const noexcept { return m_time_step; }
  Data data() const noexcept { return *m_data_ptr; }
  DataPtr data_ptr() const noexcept { return m_data_ptr; }

  // mutators
  Self& set_time_step(double t_time_step) {
    m_time_step = t_time_step;
    return *this;
  }
  Self& set_data_ptr(DataPtr t_data_ptr) {
    m_data_ptr = t_data_ptr;
    return *this;
  }

 private:
  double m_time;
  double m_time_step;
  DataPtr m_data_ptr;
};

template <typename State, typename Data, typename System>
constexpr double ModelBase<State, Data, System>::default_time_step;

template <typename State, typename Data = PointMassModelData<State>,
          typename System = PointMassModelSystem<State, Data>>
class PointMassModel : public ModelBase<State, Data, System> {
  using Self = PointMassModel;
  using DataPtr = std::shared_ptr<Data>;
  using Function = typename System::Function;

 public:
  PointMassModel();
  explicit PointMassModel(const State& t_initial_poisition);
  PointMassModel(const State& t_initial_poisition, double t_mass);
  explicit PointMassModel(DataPtr t_data_ptr);

 private:
};

template <typename State, typename Data, typename System>
PointMassModel<State, Data, System>::PointMassModel()
    : PointMassModel(State(0), Data::default_mass) {}

template <typename State, typename Data, typename System>
PointMassModel<State, Data, System>::PointMassModel(
    const State& t_initial_position)
    : PointMassModel(t_initial_position, Data::default_mass) {}

template <typename State, typename Data, typename System>
PointMassModel<State, Data, System>::PointMassModel(
    const State& t_initial_position, double t_mass)
    : ModelBase<State, Data, System>(
          createPointMassModelData(t_initial_position, t_mass)) {}

template <typename State, typename Data, typename System>
PointMassModel<State, Data, System>::PointMassModel(DataPtr t_data_ptr)
    : ModelBase<State, Data, System>(t_data_ptr) {}

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
