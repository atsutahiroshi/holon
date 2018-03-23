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

#include "holon/corelib/control/model_base.hpp"

#include <array>
#include "holon/corelib/control/system_base.hpp"
#include "holon/corelib/data/data_set_base.hpp"
#include "holon/corelib/math/ode_euler.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace experimental {
namespace {

struct TestRawData {
  double p, v;
};
struct TestData : DataSetBase<TestData, TestRawData> {
  TestData() : DataSetBase(TestRawData{0, 0}) {}
  TestData(double p, double v) : DataSetBase(TestRawData{p, v}) {}
};

struct TestSystem : SystemBase<double, TestData> {
  explicit TestSystem(TestData t_data) : SystemBase(t_data) {}
  SystemBase::StateArray operator()(const StateArray&,
                                    const double) const final {
    return StateArray{0, 1};
  };
};

struct TestModel
    : ModelBase<double, Euler<std::array<double, 2>>, TestData, TestSystem> {
  TestModel() : ModelBase(make_data<TestData>()) {}
  explicit TestModel(TestData t_data) : ModelBase(t_data) {}
  TestModel(double p, double v) : ModelBase(TestData(p, v)) {}
  virtual ~TestModel() = default;
};

void CheckMembers(const TestModel& model, const double expected_p,
                  const double expected_v) {
  CHECK(model.time() == 0.0);
  CHECK(model.time_step() == TestModel::default_time_step);
  CHECK(model.data().get().p == expected_p);
  CHECK(model.data().get().v == expected_v);
  CHECK(model.data() == model.system().data());
}

TEST_CASE("C'tors of ModelBase", "[ModelBase][ctor]") {
  SECTION("Default c'tor") {
    auto model = make_model<TestModel>();
    CheckMembers(model, 0, 0);
  }
  SECTION("Overloaded c'tor") {
    auto data = make_data<TestData>(1, 2);
    auto model = make_model<TestModel>(data);
    CHECK(model.data() == data);
    CheckMembers(model, 1, 2);
  }
}

TEST_CASE("Check accssors / mutators in ModelBase",
          "[ModelBase][accessor][mutator]") {
  SECTION("time step") {
    double dt = Fuzzer(0, 0.01).get();
    auto model = make_model<TestModel>();
    REQUIRE(model.time_step() != dt);
    model.set_time_step(dt);
    CHECK(model.time_step() == Approx(dt));
  }
  SECTION("data pointer") {
    auto data = make_data<TestData>();
    auto model = make_model<TestModel>();
    REQUIRE(model.data() != data);
    model.set_data(data);
    CHECK(model.data() == data);
  }
}

TEST_CASE("Check safely modify time step in ModelBase",
          "[ModelBase][set_time_step]") {
  auto model = make_model<TestModel>();
  SECTION("if 0 is given, modify it to default time step") {
    model.set_time_step(0);
    CHECK(model.time_step() == TestModel::default_time_step);
  }
  SECTION("if negative value is given, modify it to default time step") {
    model.set_time_step(-0.01);
    CHECK(model.time_step() == TestModel::default_time_step);
  }
}

TEST_CASE("Check update in ModelBase", "[ModelBase][update]") {
  auto model = make_model<TestModel>();
  SECTION("overloaded function 1") {
    REQUIRE(model.time() == 0.0);
    model.update();
    CHECK(model.time() == Approx(TestModel::default_time_step));
    model.update();
    CHECK(model.time() == Approx(2.0 * TestModel::default_time_step));
  }
  SECTION("overloaded function 2") {
    Fuzzer fuzz(0, 0.01);
    REQUIRE(model.time() == 0.0);
    double dt = fuzz();
    model.update(dt);
    CHECK(model.time() == Approx(dt));
    double dt2 = fuzz();
    model.update(dt2);
    CHECK(model.time() == Approx(dt + dt2));
  }
}

TEST_CASE("Check reset in ModelBase", "[ModelBase][reset]") {
  auto model = make_model<TestModel>();
  model.update();
  REQUIRE(model.time() > 0.0);
  model.reset();
  CHECK(model.time() == 0.0);
}

void CheckCopyData_1() {
  Fuzzer fuzz;
  auto model = make_model<TestModel>();
  auto data = make_data<TestData>(fuzz(), fuzz());
  REQUIRE(model.data().get().p != data.get().p);
  REQUIRE(model.data().get().v != data.get().v);
  model.copy_data(data);
  CHECK(model.data() != data);
  CHECK(model.data().get().p == data.get().p);
  CHECK(model.data().get().v == data.get().v);
}

void CheckCopyData_2() {
  Fuzzer fuzz;
  auto model1 = make_model<TestModel>();
  auto model2 = make_model<TestModel>(fuzz(), fuzz());
  REQUIRE(model1.data().get().p != model2.data().get().p);
  REQUIRE(model1.data().get().v != model2.data().get().v);
  model1.copy_data(model2);
  CHECK(model1.data() != model2.data());
  CHECK(model1.data().get().p == model2.data().get().p);
  CHECK(model1.data().get().v == model2.data().get().v);
}

TEST_CASE("Check copy_data in ModelBase", "[ModelBase][copy_data]") {
  SECTION("Overloaded function 1") { CheckCopyData_1(); }
  SECTION("Overloaded function 2") { CheckCopyData_2(); }
}

}  // namespace
}  // namespace experimental
}  // namespace holon
