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
  static_assert(HOLON_HAS_MEMBER_VAR(Data, is_data_type),
                "Data must be derived from DataSetBase class.");
  static_assert(HOLON_HAS_MEMBER_VAR(Model, is_model_type),
                "Model must be derived from ModelBase class.");
  static_assert(std::is_default_constructible<Data>::value,
                "Data must be default constructible.");

  using Self = CtrlBase<State, Solver, Data, Model>;
  using ModelPtr = std::shared_ptr<Model>;

 protected:
  using ModelDataIndex = typename Data::ModelDataIndex;
  using ParamsDataIndex = typename Data::ParamsDataIndex;
  using OutputsDataIndex = typename Data::OutputsDataIndex;
  using CommandsDataIndex = typename Data::CommandsDataIndex;
  template <std::size_t I>
  using ModelRawDataType = typename Model::template RawDataType<I>;
  template <std::size_t I>
  using ParamsRawDataType =
      typename Data::template RawDataType<ParamsDataIndex::template get<I>()>;
  template <std::size_t I>
  using OutputsRawDataType =
      typename Data::template RawDataType<OutputsDataIndex::template get<I>()>;
  template <std::size_t I>
  using CommandsRawDataType =
      typename Data::template RawDataType<CommandsDataIndex::template get<I>()>;
  template <std::size_t I>
  using CommandsRawDataPtrType = typename Data::template RawDataPtrType<
      CommandsDataIndex::template get<I>()>;

 public:
  static constexpr auto default_time_step = Model::default_time_step;

 public:
  CtrlBase() : CtrlBase(make_data<Data>()) {}
  explicit CtrlBase(const Model& t_model) : CtrlBase(make_data<Data>()) {
    this->copy_model_data(t_model);
  }
  explicit CtrlBase(Data t_data)
      : m_data(t_data),
        m_model_ptr(std::make_shared<Model>(this->model_data())) {}
  CtrlBase(Data t_data, ModelPtr t_model_ptr)
      : m_data(t_data), m_model_ptr(t_model_ptr) {
    m_model_ptr->set_data(t_data.template extract<typename Model::DataType>(
        typename decltype(t_data)::ModelDataIndex()));
  }
  virtual ~CtrlBase() = default;

  // accessors
  const Data& data() const { return m_data; }
  const Model& model() const { return *m_model_ptr; }
  Model& model() { return *m_model_ptr; }
  const ModelPtr& model_ptr() const { return m_model_ptr; }
  ModelPtr model_ptr() { return m_model_ptr; }
  double time() const { return model().time(); }
  double time_step() const { return model().time_step(); }

  // accessors to data
  template <std::size_t I = 0>
  const ModelRawDataType<I>& states() const {
    return m_model_ptr->template states<I>();
  }
  template <std::size_t I = 0>
  ModelRawDataType<I>& states() {
    return m_model_ptr->template states<I>();
  }

  typename Model::DataType model_data() const {
    return m_data.template extract<typename Model::DataType>(ModelDataIndex());
  }

  void copy_model_data(const Model& t_model) {
    m_data.template copy_subdata(t_model.data(), ModelDataIndex());
  }

  template <std::size_t I = 0>
  const ParamsRawDataType<I>& params() const {
    return m_data.template get<ParamsDataIndex::template get<I>()>();
  }
  template <std::size_t I = 0>
  ParamsRawDataType<I>& params() {
    return m_data.template get<ParamsDataIndex::template get<I>()>();
  }

  template <std::size_t I = 0>
  const OutputsRawDataType<I>& outputs() const {
    return m_data.template get<OutputsDataIndex::template get<I>()>();
  }
  template <std::size_t I = 0>
  OutputsRawDataType<I>& outputs() {
    return m_data.template get<OutputsDataIndex::template get<I>()>();
  }
  template <std::size_t I = 0>
  const CommandsRawDataType<I>& commands() const {
    return m_data.template get<CommandsDataIndex::template get<I>()>();
  }
  template <std::size_t I = 0>
  CommandsRawDataPtrType<I> get_commands_handler() const {
    return m_data.template get_ptr<CommandsDataIndex::template get<I>()>();
  }

  // mutators
  Self& set_time_step(double dt) {
    m_model_ptr->set_time_step(dt);
    return *this;
  }

  // reset
  virtual Self& reset() {
    this->m_model_ptr->reset();
    return *this;
  }

  // update functions
  virtual bool update() { return m_model_ptr->update(); }
  virtual bool update(double dt) {
    this->set_time_step(dt);
    return this->update();
  }

 private:
  Data m_data;
  ModelPtr m_model_ptr;
};

}  // namespace holon

#endif  // HOLON_CONTROL_CTRL_BASE_HPP_
