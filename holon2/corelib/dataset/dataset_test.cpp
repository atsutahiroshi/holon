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

struct TestDataUnit1 {
  double a, b;
};
struct TestDataUnit2 {
  int id;
  double x, y;
};
struct TestDataUnit3 {
  Vec3d p, v;
};
using TestDataset1 = Dataset<TestDataUnit1, TestDataUnit2>;
using TestDataset2 = Dataset<TestDataUnit1, TestDataUnit2, TestDataUnit3>;

void checkCtor(const TestDataset1& data) {
  auto ptr1 = data.getptr<0>();
  auto ptr2 = data.getptr<1>();
  CHECK(ptr1 != nullptr);
  CHECK(ptr2 != nullptr);
  CHECK(std::is_same<decltype(ptr1), std::shared_ptr<TestDataUnit1>>::value);
  CHECK(std::is_same<decltype(ptr2), std::shared_ptr<TestDataUnit2>>::value);
}
TEST_CASE("dataset: c'tor of Dataset", "[Dataset]") {
  SECTION("default c'tor") {
    TestDataset1 data;
    checkCtor(data);
  }
  SECTION("if pointers of data units are given object shares them") {
    auto ptr1 = std::make_shared<TestDataUnit1>();
    auto ptr2 = std::make_shared<TestDataUnit2>();
    TestDataset1 data(ptr1, ptr2);
    checkCtor(data);
    CHECK(data.getptr<0>() == ptr1);
    CHECK(data.getptr<1>() == ptr2);
  }
  SECTION("if tuple of data unit pointers is given object shares it") {
    auto tuple = std::make_tuple(std::make_shared<TestDataUnit1>(),
                                 std::make_shared<TestDataUnit2>());
    TestDataset1 data(tuple);
    checkCtor(data);
    CHECK(data.getptr<0>() == std::get<0>(tuple));
    CHECK(data.getptr<1>() == std::get<1>(tuple));
  }
  SECTION("if data unit instances are given object copies the contents") {
    Random<double> rnd;
    TestDataUnit1 unit1;
    TestDataUnit2 unit2;
    unit1.a = rnd();
    unit1.b = rnd();
    unit2.x = rnd();
    unit2.y = rnd();
    TestDataset1 data(unit1, unit2);
    CHECK(data.getptr<0>().get() != &unit1);
    CHECK(data.getptr<1>().get() != &unit2);
    CHECK(data.get<0>().a == unit1.a);
    CHECK(data.get<0>().b == unit1.b);
    CHECK(data.get<1>().x == unit2.x);
    CHECK(data.get<1>().y == unit2.y);
  }
}

TEST_CASE("dataset: size returns the number of given data units", "[Dataset]") {
  TestDataset1 data;
  CHECK(data.size() == 2);
}

TEST_CASE("dataset: access to an element of data unit", "[Dataset]") {
  Random<double> rnd;
  TestDataset1 data;
  auto a = rnd();
  auto x = rnd();
  data.get<0>().a = a;
  data.get<1>().x = x;
  CHECK(data.get<0>().a == a);
  CHECK(data.get<1>().x == x);
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

void checkSubDataset(const SampleDataset& data, const SubDataset1& subdata1,
                     const SubDataset2& subdata2) {
  SECTION("case 1") {
    CHECK(subdata1.getptr<0>() == data.getptr<0>());
    CHECK(subdata1.getptr<1>() == data.getptr<1>());
  }
  SECTION("case 2") {
    CHECK(subdata2.getptr<0>() == data.getptr<3>());
    CHECK(subdata2.getptr<1>() == data.getptr<1>());
    CHECK(subdata2.getptr<2>() == data.getptr<4>());
  }
}
TEST_CASE("dataset: get sub Dataset", "[Dataset]") {
  SampleDataset data;
  SECTION("overloaded function 1") {
    auto subdata1 = data.subdata<0, 1>();
    auto subdata2 = data.subdata<3, 1, 4>();
    checkSubDataset(data, subdata1, subdata2);
  }
  SECTION("overloaded function 2") {
    auto subdata1 = data.subdata(IndexSeq<0, 1>());
    auto subdata2 = data.subdata(IndexSeq<3, 1, 4>());
    checkSubDataset(data, subdata1, subdata2);
  }
}

TEST_CASE("dataset: copy contents of specific data unit", "[Dataset]") {
  Random<double> rnd;
  double a = rnd(), b = rnd();
  SECTION("overloaded function 1") {
    TestDataset1 src;
    src.get<0>().a = a;
    src.get<0>().b = b;
    TestDataset1 dst;
    dst.copy<0>(src);
    REQUIRE(dst.getptr<0>() != src.getptr<0>());
    CHECK(dst.get<0>().a == a);
    CHECK(dst.get<0>().b == b);
  }
  SECTION("overloaded function 2") {
    TestDataUnit1 src;
    src.a = a;
    src.b = b;
    TestDataset1 dst;
    dst.copy<0>(src);
    REQUIRE(dst.getptr<0>().get() != &src);
    CHECK(dst.get<0>().a == a);
    CHECK(dst.get<0>().b == b);
  }
}

TEST_CASE("dataset: copy contents of multiple data unit", "[Dataset]") {
  Random<double> rnd;
  double a = rnd(), b = rnd();
  double x = rnd(), y = rnd();
  SECTION("overloaded function 1") {
    TestDataset2 src;
    src.get<0>().a = a;
    src.get<0>().b = b;
    src.get<1>().x = x;
    src.get<1>().y = y;
    TestDataset2 dst;
    dst.copy<0, 1>(src);
    REQUIRE(dst.getptr<0>() != src.getptr<0>());
    REQUIRE(dst.getptr<1>() != src.getptr<1>());
    CHECK(dst.get<0>().a == a);
    CHECK(dst.get<0>().b == b);
    CHECK(dst.get<1>().x == x);
    CHECK(dst.get<1>().y == y);
  }
  SECTION("overloaded function 2") {
    TestDataUnit1 src1;
    TestDataUnit2 src2;
    src1.a = a;
    src1.b = b;
    src2.x = x;
    src2.y = y;
    TestDataset2 dst;
    dst.copy<0, 1>(src1, src2);
    REQUIRE(dst.getptr<0>().get() != &src1);
    REQUIRE(dst.getptr<1>().get() != &src2);
    CHECK(dst.get<0>().a == a);
    CHECK(dst.get<0>().b == b);
    CHECK(dst.get<1>().x == x);
    CHECK(dst.get<1>().y == y);
  }
}

TEST_CASE("dataset: copy contents of some data unit between different dataset",
          "[Dataset]") {
  Random<double> rnd;
  double a = rnd(), b = rnd();
  double x = rnd(), y = rnd();
  SECTION("overloaded function 1") {
    TestDataset1 src;
    src.get<0>().a = a;
    src.get<0>().b = b;
    src.get<1>().x = x;
    src.get<1>().y = y;
    TestDataset2 dst;
    dst.copy(src, IndexSeq<0, 1>(), IndexSeq<0, 1>());
    REQUIRE(dst.getptr<0>() != src.getptr<0>());
    REQUIRE(dst.getptr<1>() != src.getptr<1>());
    CHECK(dst.get<0>().a == a);
    CHECK(dst.get<0>().b == b);
    CHECK(dst.get<1>().x == x);
    CHECK(dst.get<1>().y == y);
  }
}

TEST_CASE("dataset: clone dataset instance", "[Dataset]") {
  Random<double> rnd;
  double a = rnd(), b = rnd();
  double x = rnd(), y = rnd();
  SECTION("overloaded function 1") {
    TestDataset1 org;
    org.get<0>().a = a;
    org.get<0>().b = b;
    org.get<1>().x = x;
    org.get<1>().y = y;
    TestDataset1 cln = org.clone();
    REQUIRE(cln.getptr<0>() != org.getptr<0>());
    REQUIRE(cln.getptr<1>() != org.getptr<1>());
    CHECK(cln.get<0>().a == a);
    CHECK(cln.get<0>().b == b);
    CHECK(cln.get<1>().x == x);
    CHECK(cln.get<1>().y == y);
  }
  SECTION("overloaded function 2") {
    TestDataset2 org;
    org.get<0>().a = a;
    org.get<0>().b = b;
    org.get<1>().x = x;
    org.get<1>().y = y;
    TestDataset1 cln = org.clone<0, 1>();
    REQUIRE(cln.getptr<0>() != org.getptr<0>());
    REQUIRE(cln.getptr<1>() != org.getptr<1>());
    CHECK(cln.get<0>().a == a);
    CHECK(cln.get<0>().b == b);
    CHECK(cln.get<1>().x == x);
    CHECK(cln.get<1>().y == y);
  }
}

TEST_CASE("dataset: check equality operator", "[Dataset]") {
  SECTION("case 1") {
    TestDataset2 a, b;
    CHECK(a != b);
    a = b;
    CHECK(a == b);
  }
  SECTION("case 2") {
    TestDataset2 a;
    TestDataset2 b(a);
    CHECK(a == b);
  }
  SECTION("case 3") {
    TestDataset2 a;
    auto b = a.clone();
    CHECK(a != b);
  }
}

TEST_CASE("dataset: make dataset from std::tuple") {
  using tuple1 = std::tuple<>;
  CHECK(std::is_same<DatasetFromTuple<tuple1>, Dataset<>>::value);
  using tuple2 = std::tuple<A>;
  CHECK(std::is_same<DatasetFromTuple<tuple2>, Dataset<A>>::value);
  using tuple3 = std::tuple<A, B>;
  CHECK(std::is_same<DatasetFromTuple<tuple3>, Dataset<A, B>>::value);
}

TEST_CASE("dataset: MultiDataset", "[Dataset]") {
  using Dataset1 = Dataset<A, B>;
  using Dataset2 = Dataset<C, D>;
  using Dataset3 = Dataset<E>;
  CHECK(std::is_same<MultiDataset<Dataset1>, Dataset<A, B>>::value);
  CHECK(std::is_same<MultiDataset<Dataset1, Dataset2>,
                     Dataset<A, B, C, D>>::value);
  CHECK(std::is_same<MultiDataset<Dataset1, Dataset2, Dataset3>,
                     Dataset<A, B, C, D, E>>::value);
}

}  // namespace
}  // namespace holon
