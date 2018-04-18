/* pd_ctrl - Simple PD control class
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

#ifndef HOLON_CONTROL_PD_CTRL_HPP_
#define HOLON_CONTROL_PD_CTRL_HPP_

#include <array>
#include <functional>
#include "holon/corelib/control/ctrl_base.hpp"
#include "holon/corelib/control/pd_ctrl/pd_ctrl_formula.hpp"
#include "holon/corelib/control/point_mass_model.hpp"
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

template <typename State>
struct PdCtrlParamsRawData : RawDataBase {
  double mass = 1.0;
  State position = State{0};
  State velocity = State{0};
  State stiffness = State{0};
  State damping = State{0};
  PdCtrlParamsRawData() = default;
};

template <typename State>
struct PdCtrlOutputsRawData : RawDataBase {
  State position = State{0};
  State velocity = State{0};
  State acceleration = State{0};
  State force = State{0};
  PdCtrlOutputsRawData() = default;
};

template <typename State>
class PdCtrlData : public DataSetBase<PointMassModelRawData<State>,
                                      PdCtrlParamsRawData<State>,
                                      PdCtrlOutputsRawData<State>> {
 public:
  using ModelRawData = PointMassModelRawData<State>;
  using ParamsRawData = PdCtrlParamsRawData<State>;
  using OutputsRawData = PdCtrlOutputsRawData<State>;

  HOLON_DEFINE_DEFAULT_DATA_CTOR(PdCtrlData, ModelRawData, ParamsRawData,
                                 OutputsRawData);
  using Base = DataSetBase<ModelRawData, ParamsRawData, OutputsRawData>;

 public:
  using ModelDataIndex = index_seq<0>;
  using ParamsDataIndex = index_seq<1>;
  using OutputsDataIndex = index_seq<2>;
  using CommandsDataIndex = index_seq<>;

  PdCtrlData(const State& t_initial_position, double t_mass)
      : Base(alloc_raw_data<ModelRawData>(t_mass, t_initial_position),
             alloc_raw_data<ParamsRawData>(),
             alloc_raw_data<OutputsRawData>()) {
    this->template get<1>().position = this->template get<0>().position;
    this->template get<1>().velocity = this->template get<0>().velocity;
  }
};

template <typename State, typename Solver = RungeKutta4<std::array<State, 2>>,
          typename Data = PdCtrlData<State>,
          typename Model = PointMassModel<State>>
class PdCtrl : public CtrlBase<State, Solver, Data, Model> {
  using Self = PdCtrl<State, Solver, Data, Model>;
  using Base = CtrlBase<State, Solver, Data, Model>;
  using Function = typename Model::Function;

 public:
  PdCtrl() : PdCtrl(make_data<Data>()) {}
  explicit PdCtrl(const Model& t_model) : Base(t_model) { init(); }
  explicit PdCtrl(Data t_data) : Base(t_data) { init(); }
  PdCtrl(Data t_data, std::shared_ptr<Model> t_model_ptr)
      : Base(t_data, t_model_ptr) {
    init();
  }
  virtual ~PdCtrl() = default;

  virtual State force(const State& t_position, const State& t_velocity,
                      const double /* t_time */) {
    return pd_ctrl_formula::computeDesForce(
        t_position, t_velocity, this->params().position,
        this->params().velocity, this->params().stiffness,
        this->params().damping);
  }

  virtual Function getForceFunction() {
    namespace pl = std::placeholders;
    return std::bind(&Self::force, this, pl::_1, pl::_2, pl::_3);
  }

  virtual Self& reset() override {
    this->model().reset();
    return this->resetParams();
  }
  virtual Self& reset(const State& t_initial_position) {
    this->model().reset(t_initial_position);
    return this->resetParams();
  }
  State initial_position() const { return this->model().initial_position(); }

  virtual bool update() override {
    if (!Base::update()) return false;
    updateOutputs();
    return true;
  }
  virtual bool update(double dt) override {
    this->set_time_step(dt);
    return this->update();
  }

 private:
  void init() {
    this->model().set_initial_position(this->states().position);
    resetParams();
    this->model().setForceCallback(getForceFunction());
  }
  Self& resetParams() {
    this->params().mass = this->states().mass;
    this->params().position = this->states().position;
    this->params().velocity = this->states().velocity;
    return *this;
  }
  Self& updateOutputs() {
    this->outputs().position = this->states().position;
    this->outputs().velocity = this->states().velocity;
    this->outputs().acceleration = this->states().acceleration;
    this->outputs().force = this->states().force;
    return *this;
  }
};

}  // namespace holon

#endif  // HOLON_CONTROL_PD_CTRL_HPP_
