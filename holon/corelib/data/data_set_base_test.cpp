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
  double v = default_v;
  double x = 0;
};
constexpr double RawDataSample1::default_v;

class DataSetSample1 : public DataSetBase<DataSetSample1, RawDataSample1> {
 public:
  DataSetSample1() {}
  explicit DataSetSample1(const RawDataSample1& raw_data)
      : DataSetBase<DataSetSample1, RawDataSample1>(raw_data) {}
  explicit DataSetSample1(std::shared_ptr<RawDataSample1> raw_data)
      : DataSetBase<DataSetSample1, RawDataSample1>(raw_data) {}
};

struct RawDataSample2 {
  static constexpr double default_in = 0.5;
  double in = default_in;
  double out = 0;
};
constexpr double RawDataSample2::default_in;

class DataSetSample2 : public DataSetBase<DataSetSample2, RawDataSample1,
                                          RawDataSample2, RawDataSample2> {
 public:
  DataSetSample2() {}
  DataSetSample2(const RawDataSample1& raw_data1,
                 const RawDataSample2& raw_data2,
                 const RawDataSample2& raw_data3)
      : DataSetBase<DataSetSample2, RawDataSample1, RawDataSample2,
                    RawDataSample2>(raw_data1, raw_data2, raw_data3) {}
  DataSetSample2(std::shared_ptr<RawDataSample1> raw_data1,
                 std::shared_ptr<RawDataSample2> raw_data2,
                 std::shared_ptr<RawDataSample2> raw_data3)
      : DataSetBase<DataSetSample2, RawDataSample1, RawDataSample2,
                    RawDataSample2>(raw_data1, raw_data2, raw_data3) {}
  RawDataSample1& data1() { return get<0>(); }
  RawDataSample2& data2() { return get<1>(); }
  RawDataSample2& data3() { return get<2>(); }
};

void CheckValues1(const RawDataSample1& data, const double expected_v,
                  const double expected_x) {
  CHECK(data.v == expected_v);
  CHECK(data.x == expected_x);
}

void CheckValues1(const RawDataSample1& data1, const RawDataSample1& data2) {
  CheckValues1(data1, data2.v, data2.x);
}

void CheckPtr1(const DataSetSample1& data) {
  REQUIRE(data.get_ptr<0>() != nullptr);
}

void CheckPtrIsEqual1(const DataSetSample1& data,
                      const std::shared_ptr<RawDataSample1>& ptr) {
  REQUIRE(data.get_ptr<0>() == ptr);
}

void CheckPtrIsEqual1(const DataSetSample1& data1,
                      const DataSetSample1& data2) {
  REQUIRE(data1.get_ptr<0>() == data2.get_ptr<0>());
}

void CheckPtrIsNotEqual1(const DataSetSample1& data,
                         const std::shared_ptr<RawDataSample1>& ptr) {
  REQUIRE(data.get_ptr<0>() != ptr);
}

void CheckPtrIsNotEqual1(const DataSetSample1& data1,
                         const DataSetSample1& data2) {
  REQUIRE(data1.get_ptr<0>() != data2.get_ptr<0>());
}

void CheckDataSetSample1(const DataSetSample1& data, const double expected_v,
                         const double expected_x) {
  CheckPtr1(data);
  CheckValues1(data.get<0>(), expected_v, expected_x);
}

void CheckDataSetSample1(const DataSetSample1& data,
                         const RawDataSample1& rawdata) {
  CheckDataSetSample1(data, rawdata.v, rawdata.x);
}

void CheckIfSharePtr1(const DataSetSample1& data1,
                      const DataSetSample1& data2) {
  CheckPtrIsEqual1(data1, data2);
  CheckValues1(data1.get<0>(), data2.get<0>());
}

void CheckIfNotSharePtrButEqual1(const DataSetSample1& data1,
                                 const DataSetSample1& data2) {
  CheckPtrIsNotEqual1(data1, data2);
  CheckValues1(data1.get<0>(), data2.get<0>());
}

void CheckValues2(const RawDataSample2& data, const double expected_in,
                  const double expected_out) {
  CHECK(data.in == expected_in);
  CHECK(data.out == expected_out);
}

void CheckValues2(const RawDataSample2& data1, const RawDataSample2& data2) {
  CheckValues2(data1, data2.in, data2.out);
}

void CheckPtr2(const DataSetSample2& data) {
  REQUIRE(data.get_ptr<0>() != nullptr);
}

void CheckPtrIsEqual2(const DataSetSample2& data,
                      const std::shared_ptr<RawDataSample1>& ptr1,
                      const std::shared_ptr<RawDataSample2>& ptr2,
                      const std::shared_ptr<RawDataSample2>& ptr3) {
  REQUIRE(data.get_ptr<0>() == ptr1);
  REQUIRE(data.get_ptr<1>() == ptr2);
  REQUIRE(data.get_ptr<2>() == ptr3);
}

void CheckPtrIsEqual2(const DataSetSample2& data1,
                      const DataSetSample2& data2) {
  REQUIRE(data1.get_ptr<0>() == data2.get_ptr<0>());
  REQUIRE(data1.get_ptr<1>() == data2.get_ptr<1>());
  REQUIRE(data1.get_ptr<2>() == data2.get_ptr<2>());
}

void CheckPtrIsNotEqual2(const DataSetSample2& data,
                         const std::shared_ptr<RawDataSample1>& ptr1,
                         const std::shared_ptr<RawDataSample2>& ptr2,
                         const std::shared_ptr<RawDataSample2>& ptr3) {
  REQUIRE(data.get_ptr<0>() != ptr1);
  REQUIRE(data.get_ptr<1>() != ptr2);
  REQUIRE(data.get_ptr<2>() != ptr3);
}

void CheckPtrIsNotEqual2(const DataSetSample2& data1,
                         const DataSetSample2& data2) {
  REQUIRE(data1.get_ptr<0>() != data2.get_ptr<0>());
  REQUIRE(data1.get_ptr<1>() != data2.get_ptr<1>());
  REQUIRE(data1.get_ptr<2>() != data2.get_ptr<2>());
}

void CheckDataSetSample2(const DataSetSample2& data, const double expected_v,
                         const double expected_x, const double expected_in1,
                         const double expected_out1, const double expected_in2,
                         const double expected_out2) {
  CheckPtr2(data);
  CheckValues1(data.get<0>(), expected_v, expected_x);
  CheckValues2(data.get<1>(), expected_in1, expected_out1);
  CheckValues2(data.get<2>(), expected_in2, expected_out2);
}

void CheckDataSetSample2(const DataSetSample2& data,
                         const RawDataSample1& rawdata1,
                         const RawDataSample2& rawdata2,
                         const RawDataSample2& rawdata3) {
  CheckDataSetSample2(data, rawdata1.v, rawdata1.x, rawdata2.in, rawdata2.out,
                      rawdata3.in, rawdata3.out);
}

void CheckIfSharePtr2(const DataSetSample2& data1,
                      const DataSetSample2& data2) {
  CheckPtrIsEqual2(data1, data2);
  CheckValues1(data1.get<0>(), data2.get<0>());
  CheckValues2(data1.get<1>(), data2.get<1>());
  CheckValues2(data1.get<2>(), data2.get<2>());
}

void CheckIfNotSharePtrButEqual2(const DataSetSample2& data1,
                                 const DataSetSample2& data2) {
  CheckPtrIsNotEqual2(data1, data2);
  CheckValues1(data1.get<0>(), data2.get<0>());
  CheckValues2(data1.get<1>(), data2.get<1>());
  CheckValues2(data1.get<2>(), data2.get<2>());
}

TEST_CASE("Default constructor of DataSetBase", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    DataSetSample1 data;
    CheckDataSetSample1(data, RawDataSample1::default_v, 0);
  }
  SECTION("Sample DataSet 2") {
    DataSetSample2 data;
    CheckDataSetSample2(data, RawDataSample1::default_v, 0,
                        RawDataSample2::default_in, 0,
                        RawDataSample2::default_in, 0);
  }
}

TEST_CASE("Constructor which takes RawData instance should copy its data",
          "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    RawDataSample1 raw_data;
    raw_data.v = 3;
    DataSetSample1 data(raw_data);
    CheckDataSetSample1(data, raw_data);
    raw_data.v = 4;
    CheckDataSetSample1(data, 3, 0);
  }
  SECTION("Sample DataSet 2") {
    RawDataSample1 raw_data1;
    RawDataSample2 raw_data2, raw_data3;
    raw_data1.v = 1.5;
    raw_data2.in = 2.0;
    raw_data3.in = 3.0;
    DataSetSample2 data(raw_data1, raw_data2, raw_data3);
    CheckDataSetSample2(data, raw_data1, raw_data2, raw_data3);
  }
}

TEST_CASE("Constructor which takes RawData pointer should share its data",
          "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    auto raw_data_ptr = alloc_raw_data<RawDataSample1>();
    DataSetSample1 data(raw_data_ptr);
    CheckPtrIsEqual1(data, raw_data_ptr);
  }
  SECTION("Sample DataSet 2") {
    auto raw_data_ptr1 = alloc_raw_data<RawDataSample1>();
    auto raw_data_ptr2 = alloc_raw_data<RawDataSample2>();
    auto raw_data_ptr3 = alloc_raw_data<RawDataSample2>();
    DataSetSample2 data(raw_data_ptr1, raw_data_ptr2, raw_data_ptr3);
    CheckPtrIsEqual2(data, raw_data_ptr1, raw_data_ptr2, raw_data_ptr3);
  }
}

TEST_CASE("Calling operator() in DataSet", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    DataSetSample1 data;
    data().v = 3;
    CheckDataSetSample1(data, 3, 0);
  }
  SECTION("Sample DataSet 2") {
    DataSetSample2 data;
    data().data1().v = 2;
    CheckDataSetSample2(data, 2, 0, 0.5, 0, 0.5, 0);
  }
}

TEST_CASE("Copy ctor of DataSetBase should share its data pointer",
          "[DataSetBase][ctor]") {
  SECTION("Sample Data 1") {
    DataSetSample1 data1;
    data1().v = Fuzzer().get();
    DataSetSample1 data2(data1);
    CheckIfSharePtr1(data1, data2);
  }
  SECTION("Sample Data 2") {
    DataSetSample2 data1;
    data1().get<0>().v = Fuzzer().get();
    DataSetSample2 data2(data1);
    CheckIfSharePtr2(data1, data2);
  }
}

TEST_CASE("Copy assignment of DataSetBase should share its data pointer",
          "[DataSetBase][ctor]") {
  SECTION("Sample Data 1") {
    DataSetSample1 data1;
    data1().v = Fuzzer().get();
    DataSetSample1 data2;
    data2 = data1;
    CheckIfSharePtr1(data1, data2);
  }
  SECTION("Sample Data 2") {
    DataSetSample2 data1;
    data1().get<0>().v = Fuzzer().get();
    DataSetSample2 data2;
    data2 = data1;
    CheckIfSharePtr2(data1, data2);
  }
}

TEST_CASE("Check equality operator of DataSetBase", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    DataSetSample1 data1;
    DataSetSample1 data2(data1);
    REQUIRE(data1 == data2);
    DataSetSample1 data3(data1);
    auto p = alloc_raw_data<RawDataSample1>();
    data3.get_ptr<0>() = p;
    REQUIRE(data1 != data3);
  }
}

TEST_CASE("Check copy method in DataSetBase", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    DataSetSample1 data1;
    data1().v = Fuzzer().get();
    DataSetSample1 data2;
    data2.copy(data1);
    CheckIfNotSharePtrButEqual1(data1, data2);
  }
  SECTION("Sample DataSet 2") {
    Fuzzer fuzz;
    DataSetSample2 data1;
    data1.get<0>().v = fuzz();
    data1.get<1>().in = fuzz();
    data1.get<2>().in = fuzz();
    DataSetSample2 data2;
    data2.copy(data1);
    CheckIfNotSharePtrButEqual2(data1, data2);
  }
}

TEST_CASE("Check copy method in DataSetBase which takes RawData as arguments",
          "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    Fuzzer fuzz;
    RawDataSample1 raw_data;
    raw_data.v = fuzz();
    DataSetSample1 data;
    data.copy(raw_data);
    REQUIRE(data.get_ptr<0>().get() != &raw_data);
    CheckDataSetSample1(data, raw_data);
  }
}

TEST_CASE(
    "Check copy method in DataSetBase which takes RawDataPtr as arguments",
    "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    Fuzzer fuzz;
    auto raw_data = alloc_raw_data<RawDataSample1>();
    raw_data->v = fuzz();
    DataSetSample1 data;
    data.copy(raw_data);
    CheckPtrIsNotEqual1(data, raw_data);
    CheckDataSetSample1(data, *raw_data);
  }
}

TEST_CASE("Check share_with method in DataSetBase", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    DataSetSample1 data1;
    DataSetSample1 data2;
    CheckPtrIsNotEqual1(data1, data2);
    data2.share_with(data1);
    CheckIfSharePtr1(data1, data2);
  }
  SECTION("Sample DataSet 2") {
    DataSetSample2 data1;
    DataSetSample2 data2;
    CheckPtrIsNotEqual2(data1, data2);
    data2.share_with(data1);
    CheckIfSharePtr2(data1, data2);
  }
}

TEST_CASE("Check clone method in DataSetBase", "[DataSetBase]") {
  SECTION("Sample DataSet 1") {
    Fuzzer fuzz;
    DataSetSample1 data1;
    data1().v = fuzz();
    DataSetSample1 data2(data1.clone());
    CheckIfNotSharePtrButEqual1(data1, data2);
  }
}

class DataSetSample3
    : public DataSetBase<DataSetSample3, RawDataSample1, RawDataSample2> {
 public:
  DataSetSample3(std::shared_ptr<RawDataSample1> raw_data1,
                 std::shared_ptr<RawDataSample2> raw_data2)
      : DataSetBase<DataSetSample3, RawDataSample1, RawDataSample2>(
            raw_data1, raw_data2) {}
};

TEST_CASE("Check extract method in DataSetBase", "[DataSetBase][extract]") {
  // DataSetSample2 is consisted of RawDataSample1 and two RawDataSample2s
  DataSetSample2 data_set2;
  // data_set2.extract<0> creates Data instance which has RawDataSample1,
  // which is equivalent to DataSetSample1.
  auto data_set1 = data_set2.extract<DataSetSample1, 0>();
  CHECK(data_set1.get_ptr<0>() == data_set2.get_ptr<0>());
  // DataSetSample3 is consisted of RawDataSample1 and RawDataSample2
  auto data_set3 = data_set2.extract<DataSetSample3, 0, 1>();
  CHECK(data_set3.get_ptr<0>() == data_set2.get_ptr<0>());
  CHECK(data_set3.get_ptr<1>() == data_set2.get_ptr<1>());
  data_set3 = data_set2.extract<DataSetSample3, 0, 2>();
  CHECK(data_set3.get_ptr<0>() == data_set2.get_ptr<0>());
  CHECK(data_set3.get_ptr<1>() == data_set2.get_ptr<2>());
}

}  // namespace
}  // namespace holon
