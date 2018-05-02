/* dataset - Dataset class
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of holon.
 *
 * Holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "holon2/corelib/dataset/dataset.hpp"

#include <memory>
#include <tuple>
#include <type_traits>
#include "holon2/corelib/common/random.hpp"
#include "holon2/corelib/common/utility.hpp"
#include "holon2/corelib/math/vec.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

struct TestRawData1 {
  double a, b;
};
struct TestRawData2 {
  int id;
  double x, y;
};
struct TestRawData3 {
  Vec3d p, v;
};
using TestDataset1 = Dataset<TestRawData1, TestRawData2>;
using TestDataset2 = Dataset<TestRawData1, TestRawData2, TestRawData3>;

void checkCtor(const TestDataset1& data) {
  auto ptr1 = data.getRawDataPtr<0>();
  auto ptr2 = data.getRawDataPtr<1>();
  CHECK(ptr1 != nullptr);
  CHECK(ptr2 != nullptr);
  CHECK(std::is_same<decltype(ptr1), std::shared_ptr<TestRawData1>>::value);
  CHECK(std::is_same<decltype(ptr2), std::shared_ptr<TestRawData2>>::value);
}
TEST_CASE("dataset: c'tor of Dataset", "[Dataset]") {
  SECTION("default c'tor") {
    TestDataset1 data;
    checkCtor(data);
  }
  SECTION("if pointers of raw data are given object shares them") {
    auto ptr1 = std::make_shared<TestRawData1>();
    auto ptr2 = std::make_shared<TestRawData2>();
    TestDataset1 data(ptr1, ptr2);
    checkCtor(data);
    CHECK(data.getRawDataPtr<0>() == ptr1);
    CHECK(data.getRawDataPtr<1>() == ptr2);
  }
  SECTION("if tuple of raw data pointers is given object shares it") {
    auto tuple = std::make_tuple(std::make_shared<TestRawData1>(),
                                 std::make_shared<TestRawData2>());
    TestDataset1 data(tuple);
    checkCtor(data);
    CHECK(data.getRawDataPtr<0>() == std::get<0>(tuple));
    CHECK(data.getRawDataPtr<1>() == std::get<1>(tuple));
  }
}

TEST_CASE("dataset: getRawDataNum returns number of given raw data",
          "[Dataset]") {
  TestDataset1 data;
  CHECK(data.getRawDataNum() == 2);
}

TEST_CASE("dataset: access to an element of raw data", "[Dataset]") {
  Random<double> rnd;
  TestDataset1 data;
  auto a = rnd();
  auto x = rnd();
  data.getRawData<0>().a = a;
  data.getRawData<1>().x = x;
  CHECK(data.getRawData<0>().a == a);
  CHECK(data.getRawData<1>().x == x);
}

struct A {
  double a;
};
struct B {
  double b;
};
struct C {
  double c;
};
struct D {
  double d;
};
struct E {
  double e;
};
using SampleDataset = Dataset<A, B, C, D, E>;
using SubDatasetTuple1 = std::tuple<std::shared_ptr<A>, std::shared_ptr<B>>;
using SubDataset1 = Dataset<A, B>;
using SubDatasetTuple2 =
    std::tuple<std::shared_ptr<D>, std::shared_ptr<B>, std::shared_ptr<E>>;
using SubDataset2 = Dataset<D, B, E>;

void checkSubTuple(const SampleDataset& data, SubDatasetTuple1 tuple1,
                   SubDatasetTuple2 tuple2) {
  SECTION("case 1") {
    CHECK(std::get<0>(tuple1) == data.getRawDataPtr<0>());
    CHECK(std::get<1>(tuple1) == data.getRawDataPtr<1>());
  }
  SECTION("case 2") {
    CHECK(std::get<0>(tuple2) == data.getRawDataPtr<3>());
    CHECK(std::get<1>(tuple2) == data.getRawDataPtr<1>());
    CHECK(std::get<2>(tuple2) == data.getRawDataPtr<4>());
  }
}
TEST_CASE("dataset: get sub-tuple of raw data pointers", "[Dataset]") {
  SampleDataset data;
  SECTION("overloaded function 1") {
    auto t1 = data.getRawDataPtrSubTuple<0, 1>();
    auto t2 = data.getRawDataPtrSubTuple<3, 1, 4>();
    checkSubTuple(data, t1, t2);
  }
  SECTION("overloaded function 2") {
    auto t1 = data.getRawDataPtrSubTuple(IndexSeq<0, 1>());
    auto t2 = data.getRawDataPtrSubTuple(IndexSeq<3, 1, 4>());
    checkSubTuple(data, t1, t2);
  }
}

void checkSubDataset(const SampleDataset& data, const SubDataset1& subdata1,
                     const SubDataset2& subdata2) {
  SECTION("case 1") {
    CHECK(subdata1.getRawDataPtr<0>() == data.getRawDataPtr<0>());
    CHECK(subdata1.getRawDataPtr<1>() == data.getRawDataPtr<1>());
  }
  SECTION("case 2") {
    CHECK(subdata2.getRawDataPtr<0>() == data.getRawDataPtr<3>());
    CHECK(subdata2.getRawDataPtr<1>() == data.getRawDataPtr<1>());
    CHECK(subdata2.getRawDataPtr<2>() == data.getRawDataPtr<4>());
  }
}
TEST_CASE("dataset: get sub Dataset", "[Dataset]") {
  SampleDataset data;
  SECTION("overloaded function 1") {
    auto subdata1 = data.getSubDataset<0, 1>();
    auto subdata2 = data.getSubDataset<3, 1, 4>();
    checkSubDataset(data, subdata1, subdata2);
  }
  SECTION("overloaded function 2") {
    auto subdata1 = data.getSubDataset(IndexSeq<0, 1>());
    auto subdata2 = data.getSubDataset(IndexSeq<3, 1, 4>());
    checkSubDataset(data, subdata1, subdata2);
  }
}

TEST_CASE("dataset: copy contents of specific raw data", "[Dataset]") {
  Random<double> rnd;
  double a = rnd(), b = rnd();
  SECTION("overloaded function 1") {
    TestDataset1 src;
    src.getRawData<0>().a = a;
    src.getRawData<0>().b = b;
    TestDataset1 dst;
    dst.copyRawData<0>(src);
    REQUIRE(dst.getRawDataPtr<0>() != src.getRawDataPtr<0>());
    CHECK(dst.getRawData<0>().a == a);
    CHECK(dst.getRawData<0>().b == b);
  }
  SECTION("overloaded function 2") {
    TestRawData1 src;
    src.a = a;
    src.b = b;
    TestDataset1 dst;
    dst.copyRawData<0>(src);
    REQUIRE(dst.getRawDataPtr<0>().get() != &src);
    CHECK(dst.getRawData<0>().a == a);
    CHECK(dst.getRawData<0>().b == b);
  }
}

TEST_CASE("dataset: copy contents of multiple raw data", "[Dataset]") {
  Random<double> rnd;
  double a = rnd(), b = rnd();
  double x = rnd(), y = rnd();
  SECTION("overloaded function 1") {
    TestDataset2 src;
    src.getRawData<0>().a = a;
    src.getRawData<0>().b = b;
    src.getRawData<1>().x = x;
    src.getRawData<1>().y = y;
    TestDataset2 dst;
    dst.copyRawData<0, 1>(src);
    REQUIRE(dst.getRawDataPtr<0>() != src.getRawDataPtr<0>());
    REQUIRE(dst.getRawDataPtr<1>() != src.getRawDataPtr<1>());
    CHECK(dst.getRawData<0>().a == a);
    CHECK(dst.getRawData<0>().b == b);
    CHECK(dst.getRawData<1>().x == x);
    CHECK(dst.getRawData<1>().y == y);
  }
  SECTION("overloaded function 2") {
    TestRawData1 src1;
    TestRawData2 src2;
    src1.a = a;
    src1.b = b;
    src2.x = x;
    src2.y = y;
    TestDataset2 dst;
    dst.copyRawData<0, 1>(src1, src2);
    REQUIRE(dst.getRawDataPtr<0>().get() != &src1);
    REQUIRE(dst.getRawDataPtr<1>().get() != &src2);
    CHECK(dst.getRawData<0>().a == a);
    CHECK(dst.getRawData<0>().b == b);
    CHECK(dst.getRawData<1>().x == x);
    CHECK(dst.getRawData<1>().y == y);
  }
}

TEST_CASE("dataset: copy contents of some raw data between different dataset",
          "[Dataset]") {
  Random<double> rnd;
  double a = rnd(), b = rnd();
  double x = rnd(), y = rnd();
  SECTION("overloaded function 1") {
    TestDataset1 src;
    src.getRawData<0>().a = a;
    src.getRawData<0>().b = b;
    src.getRawData<1>().x = x;
    src.getRawData<1>().y = y;
    TestDataset2 dst;
    dst.copyRawData(src, IndexSeq<0, 1>(), IndexSeq<0, 1>());
    REQUIRE(dst.getRawDataPtr<0>() != src.getRawDataPtr<0>());
    REQUIRE(dst.getRawDataPtr<1>() != src.getRawDataPtr<1>());
    CHECK(dst.getRawData<0>().a == a);
    CHECK(dst.getRawData<0>().b == b);
    CHECK(dst.getRawData<1>().x == x);
    CHECK(dst.getRawData<1>().y == y);
  }
}

}  // namespace
}  // namespace holon
