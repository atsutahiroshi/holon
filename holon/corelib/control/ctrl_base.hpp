/* ctrl_base - Base class of controller
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

#ifndef HOLON_CONTROL_CTRL_BASE_HPP_
#define HOLON_CONTROL_CTRL_BASE_HPP_

#include <memory>
#include <type_traits>
#include "holon/corelib/control/model_base.hpp"
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/math/ode_solver.hpp"

namespace holon {

template <typename State, typename Solver, typename Data, typename Model>
class CtrlBase {
  static_assert(std::is_base_of<OdeSolver<Solver>, Solver>::value,
                "Solver must be derived from OdeSolver class.");
  static_assert(is_data_type<Data>::value,
                "Data must be derived from DataSetBase class.");
  static_assert(is_model_type<Model>::value,
                "Model must be derived from ModelBase class.");
  static_assert(std::is_default_constructible<Data>::value,
                "Data must be default constructible.");

  using Self = CtrlBase<State, Solver, Data, Model>;

 protected:
  using ModelDataIndex = typename Data::ModelDataIndex;
  using RefsDataIndex = typename Data::RefsDataIndex;
  using OutputsDataIndex = typename Data::OutputsDataIndex;
  template <std::size_t I>
  using ModelRawDataType = typename Model::template RawDataType<I>;
  template <std::size_t I>
  using RefsRawDataType =
      typename Data::template RawDataType<RefsDataIndex::template get<I>()>;
  template <std::size_t I>
  using OutputsRawDataType =
      typename Data::template RawDataType<OutputsDataIndex::template get<I>()>;

 public:
  static constexpr auto default_time_step = Model::default_time_step;

 public:
  CtrlBase() : m_data(), m_model(this->model_data()) {}
  CtrlBase(const Model& t_model) : CtrlBase() {
    this->copy_model_data(t_model);
  }
  virtual ~CtrlBase() = default;

  // accessors
  const Data& data() const { return m_data; }
  const Model& model() const { return m_model; }
  Model& model() { return m_model; }
  double time() const { return model().time(); }
  double time_step() const { return model().time_step(); }

  // accessors to data
  template <std::size_t I = 0>
  const ModelRawDataType<I>& states() const {
    return m_model.template states<I>();
  }
  template <std::size_t I = 0>
  ModelRawDataType<I>& states() {
    return m_model.template states<I>();
  }

  typename Model::DataType model_data() const {
    return m_data.template extract<typename Model::DataType>(ModelDataIndex());
  }

  void copy_model_data(const Model& t_model) {
    m_data.template copy_subdata(t_model.data(), ModelDataIndex());
  }

  template <std::size_t I = 0>
  const RefsRawDataType<I>& refs() const {
    return m_data.template get<RefsDataIndex::template get<I>()>();
  }
  template <std::size_t I = 0>
  RefsRawDataType<I>& refs() {
    return m_data.template get<RefsDataIndex::template get<I>()>();
  }

  template <std::size_t I = 0>
  const OutputsRawDataType<I>& outputs() const {
    return m_data.template get<OutputsDataIndex::template get<I>()>();
  }
  template <std::size_t I = 0>
  OutputsRawDataType<I>& outputs() {
    return m_data.template get<OutputsDataIndex::template get<I>()>();
  }

  // mutators
  Self& set_time_step(double dt) {
    m_model.set_time_step(dt);
    return *this;
  }

  // reset
  virtual Self& reset() {
    this->m_model.reset();
    return *this;
  }

  // update functions
  virtual bool update() { return m_model.update(); }
  virtual bool update(double dt) {
    this->set_time_step(dt);
    return this->update();
  }

 private:
  Data m_data;
  Model m_model;
};

}  // namespace holon

#endif  // HOLON_CONTROL_CTRL_BASE_HPP_
