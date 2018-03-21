/* data_set_base - Base class for data set class
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

#include "holon/corelib/data/data_set_base.hpp"

#include <memory>

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

struct RawDataSample1 {
  static constexpr double default_v = 1.0;
  double v;
  double x;

  RawDataSample1() : v(default_v), x(0.0) {}
};

struct RawDataSample2 {
  static constexpr double default_in = 0.5;
  double in;
  double out;

  RawDataSample2() : in(default_in), out(0.0) {}
};

class SampleDataSet : public DataSetBase<SampleDataSet, RawDataSample1> {
 public:
  SampleDataSet() {}
  explicit SampleDataSet(const RawDataSample1& raw_data)
      : DataSetBase<SampleDataSet, RawDataSample1>(raw_data) {}
  explicit SampleDataSet(std::shared_ptr<RawDataSample1> raw_data)
      : DataSetBase<SampleDataSet, RawDataSample1>(raw_data) {}
};

class MoreSampleDataSet : public DataSetBase<MoreSampleDataSet, RawDataSample1,
                                             RawDataSample2, RawDataSample2> {
 public:
  MoreSampleDataSet() {}
  MoreSampleDataSet(const RawDataSample1& raw_data1,
                    const RawDataSample2& raw_data2,
                    const RawDataSample2& raw_data3)
      : DataSetBase<MoreSampleDataSet, RawDataSample1, RawDataSample2,
                    RawDataSample2>(raw_data1, raw_data2, raw_data3) {}
  MoreSampleDataSet(std::shared_ptr<RawDataSample1> raw_data1,
                    std::shared_ptr<RawDataSample2> raw_data2,
                    std::shared_ptr<RawDataSample2> raw_data3)
      : DataSetBase<MoreSampleDataSet, RawDataSample1, RawDataSample2,
                    RawDataSample2>(raw_data1, raw_data2, raw_data3) {}
  RawDataSample1& data1() { return get<0>(); }
  RawDataSample2& data2() { return get<1>(); }
  RawDataSample2& data3() { return get<2>(); }
};

TEST_CASE("Default constructor of DataSetBase", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    SampleDataSet data;
    REQUIRE(data.get_ptr<0>() != nullptr);
    CHECK(data.get<0>().v == 1.0);
  }
  SECTION("Sample DataSet 2") {
    MoreSampleDataSet data;
    REQUIRE(data.get_ptr<0>() != nullptr);
    REQUIRE(data.get_ptr<1>() != nullptr);
    REQUIRE(data.get_ptr<2>() != nullptr);
    CHECK(data.get<0>().v == 1.0);
    CHECK(data.get<1>().in == 0.5);
    CHECK(data.get<2>().in == 0.5);
  }
}

TEST_CASE("Constructor which takes RawData instance should copy its data",
          "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    RawDataSample1 raw_data;
    raw_data.v = 3;
    SampleDataSet data(raw_data);
    REQUIRE(data.get_ptr<0>() != nullptr);
    CHECK(data.get<0>().v == 3.0);
    raw_data.v = 4;
    CHECK(data.get<0>().v == 3.0);
  }
  SECTION("Sample DataSet 2") {
    RawDataSample1 raw_data1;
    RawDataSample2 raw_data2, raw_data3;
    raw_data1.v = 1.5;
    raw_data2.in = 2.0;
    raw_data3.in = 3.0;
    MoreSampleDataSet data(raw_data1, raw_data2, raw_data3);
    REQUIRE(data.get_ptr<0>() != nullptr);
    REQUIRE(data.get_ptr<1>() != nullptr);
    REQUIRE(data.get_ptr<2>() != nullptr);
    CHECK(data.get<0>().v == 1.5);
    CHECK(data.get<1>().in == 2.0);
    CHECK(data.get<2>().in == 3.0);
  }
}

TEST_CASE("Constructor which takes RawData pointer should share its data",
          "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    auto raw_data_ptr = alloc_raw_data<RawDataSample1>();
    SampleDataSet data(raw_data_ptr);
    REQUIRE(data.get_ptr<0>() == raw_data_ptr);
  }
  SECTION("Sample DataSet 2") {
    auto raw_data_ptr1 = alloc_raw_data<RawDataSample1>();
    auto raw_data_ptr2 = alloc_raw_data<RawDataSample2>();
    auto raw_data_ptr3 = alloc_raw_data<RawDataSample2>();
    MoreSampleDataSet data(raw_data_ptr1, raw_data_ptr2, raw_data_ptr3);
    REQUIRE(data.get_ptr<0>() == raw_data_ptr1);
    REQUIRE(data.get_ptr<1>() == raw_data_ptr2);
    REQUIRE(data.get_ptr<2>() == raw_data_ptr3);
  }
}

TEST_CASE("Calling operator() in DataSet", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    SampleDataSet data;
    data().v = 3;
    CHECK(data.get<0>().v == 3);
  }
  SECTION("Sample DataSet 2") {
    MoreSampleDataSet data;
    data().data1().v = 2;
    CHECK(data.get<0>().v == 2);
  }
}

TEST_CASE("Copy ctor of DataSetBase should share its data pointer",
          "[DataSetBase][ctor]") {
  SECTION("Sample Data 1") {
    SampleDataSet data1;
    data1().v = Fuzzer().get();
    SampleDataSet data2(data1);
    REQUIRE(data1.get_ptr<0>() == data2.get_ptr<0>());
    CHECK(data1().v == data2().v);
  }
  SECTION("Sample Data 2") {
    MoreSampleDataSet data1;
    data1().get<0>().v = Fuzzer().get();
    MoreSampleDataSet data2(data1);
    REQUIRE(data1.get_ptr<0>() == data2.get_ptr<0>());
    REQUIRE(data1.get_ptr<1>() == data2.get_ptr<1>());
    REQUIRE(data1.get_ptr<2>() == data2.get_ptr<2>());
    CHECK(data1().get<0>().v == data2().get<0>().v);
  }
}

TEST_CASE("Check copy method in DataSetBase", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    SampleDataSet data1;
    data1().v = Fuzzer().get();
    SampleDataSet data2;
    REQUIRE(data2().v != data1().v);
    data2.copy(data1);
    CHECK(data2().v == data1().v);
    CHECK(data2.get_ptr<0>() != data1.get_ptr<0>());
  }
  SECTION("Sample DataSet 2") {
    Fuzzer fuzz;
    MoreSampleDataSet data1;
    data1.get<0>().v = fuzz();
    data1.get<1>().in = fuzz();
    data1.get<2>().in = fuzz();
    MoreSampleDataSet data2;
    REQUIRE(data2().get<0>().v != data1().get<0>().v);
    REQUIRE(data2().get<1>().in != data1().get<1>().in);
    REQUIRE(data2().get<2>().in != data1().get<2>().in);
    data2.copy(data1);
    CHECK(data2().get<0>().v == data1().get<0>().v);
    CHECK(data2().get<1>().in == data1().get<1>().in);
    CHECK(data2().get<2>().in == data1().get<2>().in);
    CHECK(data2.get_ptr<0>() != data1.get_ptr<0>());
    CHECK(data2.get_ptr<1>() != data1.get_ptr<1>());
    CHECK(data2.get_ptr<2>() != data1.get_ptr<2>());
  }
}

TEST_CASE("Check copy method in DataSetBase which takes RawData as arguments",
          "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    Fuzzer fuzz;
    RawDataSample1 raw_data;
    raw_data.v = fuzz();
    SampleDataSet data;
    REQUIRE(data.get<0>().v != raw_data.v);
    data.copy(raw_data);
    REQUIRE(data.get_ptr<0>().get() != &raw_data);
    CHECK(data.get<0>().v == raw_data.v);
  }
}

TEST_CASE(
    "Check copy method in DataSetBase which takes RawDataPtr as arguments",
    "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    Fuzzer fuzz;
    auto raw_data = alloc_raw_data<RawDataSample1>();
    raw_data->v = fuzz();
    SampleDataSet data;
    REQUIRE(data.get<0>().v != raw_data->v);
    data.copy(raw_data);
    REQUIRE(data.get_ptr<0>() != raw_data);
    CHECK(data.get<0>().v == raw_data->v);
  }
}

TEST_CASE("Check share_with method in DataSetBase", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    SampleDataSet data1;
    SampleDataSet data2;
    REQUIRE(data2.get_ptr<0>() != data1.get_ptr<0>());
    data2.share_with(data1);
    CHECK(data2.get_ptr<0>() == data1.get_ptr<0>());
  }
  SECTION("Sample DataSet 2") {
    MoreSampleDataSet data1;
    MoreSampleDataSet data2;
    REQUIRE(data2.get_ptr<0>() != data1.get_ptr<0>());
    REQUIRE(data2.get_ptr<1>() != data1.get_ptr<1>());
    REQUIRE(data2.get_ptr<2>() != data1.get_ptr<2>());
    data2.share_with(data1);
    CHECK(data2.get_ptr<0>() == data1.get_ptr<0>());
    CHECK(data2.get_ptr<1>() == data1.get_ptr<1>());
    CHECK(data2.get_ptr<2>() == data1.get_ptr<2>());
  }
}

TEST_CASE("Check clone method in DataSetBase", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    Fuzzer fuzz;
    SampleDataSet data1;
    data1().v = fuzz();
    SampleDataSet data2(data1.clone());
    REQUIRE(data2.get_ptr<0>() != data1.get_ptr<0>());
    CHECK(data2().v == data1().v);
  }
}

}  // namespace
}  // namespace holon
