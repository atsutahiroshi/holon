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

#include "holon/corelib/control/ctrl_base.hpp"

#include <array>
#include <memory>
#include "holon/corelib/control/model_base.hpp"
#include "holon/corelib/control/system_base.hpp"
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/math/ode_euler.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

struct TestModelRawData : RawDataBase {
  double p, v;
  TestModelRawData() = default;
  TestModelRawData(double t_p, double t_v) : p(t_p), v(t_v) {}
};
struct TestModelData : DataSetBase<TestModelRawData> {
  TestModelData() : DataSetBase(TestModelRawData{0, 0}) {}
  explicit TestModelData(std::shared_ptr<TestModelRawData> t_raw_data_p)
      : DataSetBase(t_raw_data_p) {}
  TestModelData(double p, double v) : DataSetBase(TestModelRawData{p, v}) {}
};

struct TestSystem : SystemBase<double, TestModelData> {
  explicit TestSystem(TestModelData t_data) : SystemBase(t_data) {}
  SystemBase::StateArray operator()(const StateArray&,
                                    const double) const final {
    return StateArray{0, 1};
  };
};

struct TestModel : ModelBase<double, Euler<std::array<double, 2>>,
                             TestModelData, TestSystem> {
  TestModel() : ModelBase(make_data<TestModelData>()) {}
  explicit TestModel(TestModelData t_data) : ModelBase(t_data) {}
  TestModel(double p, double v) : ModelBase(TestModelData(p, v)) {}
  virtual ~TestModel() = default;
};

struct TestParamsRawData : RawDataBase {
  double pd, vd;
};

struct TestOutputsRawData : RawDataBase {
  double pp, vv;
};

struct TestCtrlData
    : DataSetBase<TestModelRawData, TestParamsRawData, TestOutputsRawData> {
  TestCtrlData() {}
  TestCtrlData(std::shared_ptr<TestModelRawData> t_model_data_p,
               std::shared_ptr<TestParamsRawData> t_params_data_p,
               std::shared_ptr<TestOutputsRawData> t_outputs_data_p) {}
  using ModelDataIndex = index_seq<0>;
  using ParamsDataIndex = index_seq<1>;
  using OutputsDataIndex = index_seq<2>;
  using CommandsDataIndex = index_seq<>;
};

class TestCtrl : public CtrlBase<double, Euler<std::array<double, 2>>,
                                 TestCtrlData, TestModel> {
 public:
  TestCtrl() {}
  explicit TestCtrl(const TestModel& t_model) : CtrlBase(t_model) {}
  explicit TestCtrl(TestCtrlData t_data) : CtrlBase(t_data) {}
  TestCtrl(TestCtrlData t_data, std::shared_ptr<TestModel> t_model_ptr)
      : CtrlBase(t_data, t_model_ptr) {}

 private:
};

void CheckCtor_0() {
  TestCtrl ctrl;
  REQUIRE(std::is_same<std::decay<decltype(ctrl.data())>::type,
                       TestCtrlData>::value);
  CHECK(ctrl.data().get_ptr<0>() != nullptr);  // model data
  CHECK(ctrl.data().get_ptr<1>() != nullptr);  // params data
  CHECK(ctrl.data().get_ptr<2>() != nullptr);  // outputs data
  CHECK(ctrl.model().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
  CHECK(&ctrl.states() == &ctrl.model().states());
}

void CheckCtor_1() {
  Fuzzer fuzz;
  TestModel model;
  model.states().p = fuzz();
  model.states().v = fuzz();

  TestCtrl ctrl(model);
  REQUIRE(ctrl.data().get_ptr<0>() != model.data().get_ptr<0>());
  CHECK(ctrl.model().states().p == model.states().p);
  CHECK(ctrl.model().states().v == model.states().v);
  CHECK(ctrl.states().p == model.states().p);
  CHECK(ctrl.states().v == model.states().v);
  CHECK(ctrl.model().data().get_ptr<0>() == ctrl.data().get_ptr<0>());
}

void CheckCtor_2() {
  auto data = make_data<TestCtrlData>();
  TestCtrl ctrl(data);
  REQUIRE(ctrl.data().get_ptr<0>() == data.get_ptr<0>());
  REQUIRE(ctrl.data().get_ptr<0>() == ctrl.model().data().get_ptr<0>());
  REQUIRE(ctrl.data().get_ptr<1>() == data.get_ptr<1>());
  REQUIRE(ctrl.data().get_ptr<2>() == data.get_ptr<2>());
  CHECK(&ctrl.states() == &ctrl.model().states());
}

void CheckCtor_3() {
  auto data = make_data<TestCtrlData>();
  auto model_ptr = std::make_shared<TestModel>();
  TestCtrl ctrl(data, model_ptr);
  REQUIRE(ctrl.data().get_ptr<0>() == data.get_ptr<0>());
  REQUIRE(ctrl.data().get_ptr<0>() == ctrl.model().data().get_ptr<0>());
  REQUIRE(ctrl.data().get_ptr<1>() == data.get_ptr<1>());
  REQUIRE(ctrl.data().get_ptr<2>() == data.get_ptr<2>());
  CHECK(&ctrl.states() == &ctrl.model().states());
  CHECK(&ctrl.model() == model_ptr.get());
}

TEST_CASE("Check c'tor of Ctrlbase", "[CtrlBase][ctor]") {
  SECTION("Default c'tor") { CheckCtor_0(); }
  SECTION("Overloaded c'tor 1") { CheckCtor_1(); }
  SECTION("Overloaded c'tor 2") { CheckCtor_2(); }
  SECTION("Overloaded c'tor 3") { CheckCtor_3(); }
}

TEST_CASE("Accessors / mutators in CtrlBase", "[CtrlBase][accessor][mutator]") {
  SECTION("time step") {
    TestCtrl ctrl;
    double dt = Fuzzer(0, 0.01).get();
    REQUIRE(ctrl.time_step() != dt);
    ctrl.set_time_step(dt);
    CHECK(ctrl.time_step() == dt);
  }
}

TEST_CASE("Check update in CtrlBase", "[CtrlBase][update]") {
  TestCtrl ctrl;
  SECTION("Overloaded function 1") {
    const auto dt = TestCtrl::default_time_step;
    REQUIRE(ctrl.time() == Approx(0.0));
    CHECK(ctrl.update());
    CHECK(ctrl.time() == Approx(dt));
    CHECK(ctrl.update());
    CHECK(ctrl.time() == Approx(2.0 * dt));
  }
  SECTION("Overloaded function 2") {
    Fuzzer fuzz(0, 0.01);
    REQUIRE(ctrl.time() == Approx(0.0));
    double dt = fuzz();
    ctrl.update(dt);
    CHECK(ctrl.time_step() == Approx(dt));
    CHECK(ctrl.time() == Approx(dt));
    double dt2 = fuzz();
    ctrl.update(dt2);
    CHECK(ctrl.time_step() == Approx(dt2));
    CHECK(ctrl.time() == Approx(dt + dt2));
  }
}

TEST_CASE("Check reset in CtrlBase", "[CtrlBase][reset]") {
  TestCtrl ctrl;
  REQUIRE(ctrl.update());
  REQUIRE(ctrl.time() != 0.0);
  ctrl.reset();
  CHECK(ctrl.time() == 0.0);
}

namespace testing {

struct A : RawDataBase {
  double a;
};
struct B : RawDataBase {
  double b;
};
struct C : RawDataBase {};
struct D : RawDataBase {};
struct E : RawDataBase {};
struct F : RawDataBase {};
struct G : RawDataBase {};
struct H : RawDataBase {};
struct ModelData : DataSetBase<A, B> {
  using Base = DataSetBase<A, B>;
  explicit ModelData(typename Base::RawDataPtrTuple raw_data_ptrs)
      : Base(raw_data_ptrs) {}
  ModelData() {}
  ModelData(std::shared_ptr<A> a, std::shared_ptr<B> b) : Base(a, b) {}
};
struct Model : ModelBase<double, Euler<std::array<double, 2>>, ModelData> {
  Model() : ModelBase(make_data<ModelData>()) {}
  explicit Model(ModelData t_data) : ModelBase(t_data) {}
};
struct ParamsData : DataSetBase<C, E> {
  using Base = DataSetBase<C, E>;
  template <typename... Args>
  explicit ParamsData(std::shared_ptr<Args>... args) : Base(args...) {}
};
struct OutputsData : DataSetBase<D, F> {
  using Base = DataSetBase<D, F>;
  template <typename... Args>
  explicit OutputsData(std::shared_ptr<Args>... args) : Base(args...) {}
};

struct Data : DataSetBase<A, B, C, D, E, F, G, H> {
  using Base = DataSetBase<A, B, C, D, E, F, G, H>;
  Data() {}
  explicit Data(typename Base::RawDataPtrTuple raw_data_ptrs)
      : Base(raw_data_ptrs) {}
  using ModelDataIndex = index_seq<0, 1>;
  using ParamsDataIndex = index_seq<2, 4>;
  using OutputsDataIndex = index_seq<3, 5>;
  using CommandsDataIndex = index_seq<6, 7>;
};
class Ctrl
    : public CtrlBase<double, Euler<std::array<double, 2>>, Data, Model> {
 public:
  Ctrl() {}
  explicit Ctrl(const Model& t_model) : CtrlBase(t_model) {}
};

}  // namespace testing

TEST_CASE("Check constructor of CtrlBase whether it copies data in model",
          "[CtrlBase]") {
  using testing::Model;
  using testing::Ctrl;

  Fuzzer fuzz;
  Model model;
  model.data().get<0>().a = fuzz();
  model.data().get<1>().b = fuzz();
  Ctrl ctrl(model);
  REQUIRE(ctrl.model().data() != model.data());
  CHECK(ctrl.data().get<0>().a == model.data().get<0>().a);
  CHECK(ctrl.data().get<1>().b == model.data().get<1>().b);
}

TEST_CASE("CtrlBase::model_data() returns data that its model has",
          "[CtrlBase][model_data]") {
  using testing::Model;
  using testing::Ctrl;

  Ctrl ctrl;
  CHECK(ctrl.model_data() == ctrl.model().data());
}

TEST_CASE("CtrlBase::params() returns reference data",
          "[CtrlBase][params_data]") {
  using testing::Model;
  using testing::Ctrl;

  Ctrl ctrl;
  CHECK(&ctrl.params<0>() == &ctrl.data().get<2>());
  CHECK(&ctrl.params<1>() == &ctrl.data().get<4>());
}

TEST_CASE("CtrlBase::commands() returns commands data",
          "[CtrlBase][commands]") {
  using testing::Ctrl;

  Ctrl ctrl;
  CHECK(&ctrl.commands<0>() == &ctrl.data().get<6>());
  CHECK(&ctrl.commands<1>() == &ctrl.data().get<7>());
}

TEST_CASE("CtrlBase::get_commands_handler() returns handler to commands",
          "[CtrlBase][get_commands_handler]") {
  using testing::Ctrl;
  Ctrl ctrl;
  auto cmd1 = ctrl.get_commands_handler();
  CHECK(cmd1 == ctrl.data().get_ptr<6>());
  auto cmd2 = ctrl.get_commands_handler<1>();
  CHECK(cmd2 == ctrl.data().get_ptr<7>());
}

}  // namespace
}  // namespace holon
