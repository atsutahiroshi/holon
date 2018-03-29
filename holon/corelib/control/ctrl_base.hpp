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

namespace experimental {

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

 protected:
  using Self = CtrlBase<State, Solver, Data, Model>;
  using ModelDataIndex = typename Data::ModelDataIndex;
  using RefsDataIndex = typename Data::RefsDataIndex;
  using OutputsDataIndex = typename Data::OutputsDataIndex;

 public:
  static constexpr auto default_time_step = Model::default_time_step;

 public:
  CtrlBase()
      : m_data(),
        m_model(m_data.template extract<typename Model::DataType>(
            ModelDataIndex())) {}
  CtrlBase(const Model& t_model) : CtrlBase() {
    m_data.template copy_subdata(t_model.data(), t_model.data().index,
                                 ModelDataIndex());
  }

  // accessors
  const Data& data() const { return m_data; }
  const Model& model() const { return m_model; }
  Model& model() { return m_model; }
  double time() const { return model().time(); }
  double time_step() const { return model().time_step(); }

  // accessors to data
  template <std::size_t I = 0>
  const typename Model::template RawDataType<I>& states() const {
    return m_model.template states<I>();
  }
  template <std::size_t I = 0>
  typename Model::template RawDataType<I>& states() {
    return m_model.template states<I>();
  }

  typename Model::DataType model_data() const {
    return m_data.template extract<typename Model::DataType>(ModelDataIndex());
  }

  template <std::size_t I = 0>
  const typename Data::template RawDataI<RefsDataIndex::template get<I>()>&
  refs() const {
    return m_data.template get<RefsDataIndex::template get<I>()>();
  }
  template <std::size_t I = 0>
  typename Data::template RawDataI<RefsDataIndex::template get<I>()>& refs() {
    return m_data.template get<RefsDataIndex::template get<I>()>();
  }

  template <std::size_t I = 0>
  const typename Data::template RawDataI<OutputsDataIndex::template get<I>()>&
  outputs() const {
    return m_data.template get<OutputsDataIndex::template get<I>()>();
  }
  template <std::size_t I = 0>
  typename Data::template RawDataI<OutputsDataIndex::template get<I>()>&
  outputs() {
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

}  // namespace experimental

template <typename State, typename Solver, typename Data, typename Model,
          typename Refs, typename Outputs>
class CtrlBase {
 protected:
  using Self = CtrlBase<State, Solver, Data, Model, Refs, Outputs>;
  using States = Data;
  using StatesPtr = std::shared_ptr<States>;
  using RefsPtr = std::shared_ptr<Refs>;
  using OutputsPtr = std::shared_ptr<Outputs>;

 public:
  CtrlBase()
      : m_model(),
        m_states_ptr(m_model.data_ptr()),
        m_refs_ptr(std::make_shared<Refs>()),
        m_outputs_ptr(std::make_shared<Outputs>()) {}
  explicit CtrlBase(const Model& t_model) : CtrlBase() {
    *m_states_ptr = t_model.data();
  }
  virtual ~CtrlBase() = default;

  // accessors
  const Model& model() const { return m_model; }
  Model& model() { return m_model; }
  const States& states() const { return *m_states_ptr; }
  States& states() { return *m_states_ptr; }
  const Refs& refs() const { return *m_refs_ptr; }
  const Outputs& outputs() const { return *m_outputs_ptr; }
  const StatesPtr& states_ptr() const { return m_states_ptr; }
  const RefsPtr& refs_ptr() const { return m_refs_ptr; }
  const OutputsPtr& outputs_ptr() const { return m_outputs_ptr; }

  double time() const { return model().time(); }
  double time_step() const { return model().time_step(); }

  // mutators
  Self& set_states_ptr(StatesPtr t_states_ptr) {
    m_states_ptr = t_states_ptr;
    m_model.set_data_ptr(t_states_ptr);
    return *this;
  }
  Self& set_refs_ptr(RefsPtr t_refs_ptr) {
    m_refs_ptr = t_refs_ptr;
    return *this;
  }
  Self& set_outputs_ptr(OutputsPtr t_outputs_ptr) {
    m_outputs_ptr = t_outputs_ptr;
    return *this;
  }
  Self& set_time_step(double dt) {
    m_model.set_time_step(dt);
    return *this;
  }

  // update functions
  virtual bool update() { return m_model.update(); }
  virtual bool update(double dt) {
    this->set_time_step(dt);
    return this->update();
  }

 private:
  Model m_model;
  StatesPtr m_states_ptr;
  RefsPtr m_refs_ptr;
  OutputsPtr m_outputs_ptr;
};

}  // namespace holon

#endif  // HOLON_CONTROL_CTRL_BASE_HPP_
