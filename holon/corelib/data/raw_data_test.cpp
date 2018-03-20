/* raw_data - Raw data class
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

#include "holon/corelib/data/raw_data.hpp"

#include <memory>
#include <tuple>

#include "catch.hpp"

namespace holon {
namespace {

TEST_CASE("Data usage") {
  ComZmpModelData data;
  CHECK(data().com_position == Vec3D{0, 0, 1});

  ComZmpModelData data2(Vec3D{1, 2, 3});
  CHECK(data2().com_position == Vec3D{1, 2, 3});
}

TEST_CASE("BipedModelData") {
  BipedModelData data;
  CHECK(data().com().com_position == Vec3D{0, 0, 1});
}

TEST_CASE("BipedModelData2") {
  BipedModelData2 data;
  CHECK(data().com().com_position == Vec3D{0, 0, 1});
}

struct A {
  Vec3D p;
};

TEST_CASE("Copy shared data") {
  SharedData<A> data1(alloc_data<A>());
  data1.get().p = Vec3D{1, 2, 3};

  SharedData<A> data2(alloc_data<A>());
  // data2 = data1;
  data2 = data1.clone();

  REQUIRE(data1.ptr() != data2.ptr());
  CHECK(data2.get().p == Vec3D{1, 2, 3});
  data2.get().p = Vec3D{2, 3, 4};
  CHECK(data1.get().p == Vec3D{1, 2, 3});
  CHECK(data2.get().p == Vec3D{2, 3, 4});

  // SharedData<A> data3(data1);
  // REQUIRE(data1.ptr() != data3.ptr());

  SharedData<A> data4(data1.clone());
  REQUIRE(data1.ptr() != data4.ptr());

  SharedData<A> data5 = data1.clone();
  REQUIRE(data1.ptr() != data5.ptr());
}

TEST_CASE("Copy BipedModel") {
  BipedModelData data1;
  data1.com().com_position = Vec3D{1, 2, 3};
  BipedModelData data2 = data1.clone();
  BipedModelData data3;
  data3 = data1.clone();
  // data3 = data1;

  CHECK(data2.com().com_position == Vec3D{1, 2, 3});
  data2.com().com_position = Vec3D{2, 3, 4};
  CHECK(data1.com().com_position == Vec3D{1, 2, 3});
  CHECK(data2.com().com_position == Vec3D{2, 3, 4});
}

struct RawDataA {
  double mass;
  Vec3D pos;
  Vec3D vel;
  Vec3D acc;
};

struct RawDataB {
  double mass;
  Vec3D pos;
  Vec3D vel;
  Vec3D acc;
};

class DataA {
  SharedData<RawDataA> m_data;

 public:
  DataA() : m_data(alloc_data<RawDataA>()) {}
  DataA(const DataA& other) : m_data(other.data().ptr()) {}
  explicit DataA(SharedData<RawDataA> t_data) : m_data(t_data.ptr()) {}
  RawDataA& operator()() { return m_data.get(); }
  const RawDataA& operator()() const { return m_data.get(); }
  const SharedData<RawDataA>& data() const { return m_data; }
};

class DataB {
  SharedData<RawDataB> m_data;

 public:
  DataB() : m_data(alloc_data<RawDataB>()) {}
  DataB(const DataB& other) : m_data(other.data().ptr()) {}
  explicit DataB(SharedData<RawDataB> t_data) : m_data(t_data.ptr()) {}
  RawDataB& operator()() { return m_data.get(); }
  const RawDataB& operator()() const { return m_data.get(); }
  const SharedData<RawDataB>& data() const { return m_data; }
};

class DataAB {
  SharedData<RawDataA> m_data_a;
  SharedData<RawDataB> m_data_b;

 public:
  DataAB()
      : m_data_a(alloc_data<RawDataA>()), m_data_b(alloc_data<RawDataB>()) {}
  const SharedData<RawDataA>& data_a() { return m_data_a; }
  const SharedData<RawDataB>& data_b() { return m_data_b; }
};

class DataAB2 {
  DataA m_data_a;
  DataB m_data_b;

 public:
  DataAB2() {}
  const DataA& data_a() { return m_data_a; }
  const DataB& data_b() { return m_data_b; }
};

DataA updateA(const DataA& data) {
  DataA res;
  res().acc = data().pos + data().vel * 0.1;
  return res;
}

bool updateA2(DataA data) {
  data().acc = data().pos + data().vel * 0.1;
  return true;
}

bool updateB(DataB data) {
  data().acc = data().pos + data().vel * 0.1;
  return true;
}

bool updateAB(DataAB data) {
  // updateA(DataA(data.data_a()));
  // updateB(data.data_b());
  return true;
}
bool updateAB2(DataAB2 data) {
  updateA(data.data_a());
  updateB(data.data_b());
  return true;
}

TEST_CASE("usage of shared data") {
  DataA a1, a2;
  updateA2(a1);
}

namespace test {

struct RawDataA {
  Vec3D pos;
};

struct RawDataB {
  Vec3D pos;
};

template <typename Derived, typename... Ts>
struct DataBase {
  using DataTuple = std::tuple<std::shared_ptr<Ts>...>;
  template <std::size_t N>
  using DataType = typename std::tuple_element<N, std::tuple<Ts...>>::type;
  static constexpr bool more_than_one = sizeof...(Ts)-1;

  DataTuple m_data_tuple;
  DataTuple& data_tuple() { return m_data_tuple; }

  template <std::size_t N>
  DataType<N>& get() {
    return *std::get<N>(m_data_tuple);
  }
  template <std::size_t N>
  const DataType<N>& get() const {
    return *std::get<N>(m_data_tuple);
  }

  Derived& helper(std::true_type) { return *static_cast<Derived*>(this); }
  DataType<0>& helper(std::false_type) { return this->get<0>(); }

  typename std::conditional<more_than_one, Derived, DataType<0>>::type&
  operator()() {
    return this->helper(std::integral_constant<bool, more_than_one>());
  }

  DataBase() : m_data_tuple() {}
  explicit DataBase(Ts... args) : m_data_tuple(std::make_tuple(args...)) {}
  explicit DataBase(std::shared_ptr<Ts>... args)
      : m_data_tuple(std::make_tuple(std::make_shared<Ts>(args)...)) {}
};

struct DataTest : public DataBase<DataTest, RawDataA> {
  RawDataA a;
};
struct DataTest2 : public DataBase<DataTest2, RawDataA, RawDataB> {
  RawDataA a;
  RawDataB b;
};

TEST_CASE("data base test") {
  DataTest data;
  // data.get<0>().pos = {0, 1, 2};
  // data().pos = {1, 2, 3};
  // data.get<0>()->pos = {0, 1, 2};
  DataTest2 data2;
  // data2().a.pos = {2, 3, 4};
}

struct DataA {
  std::shared_ptr<RawDataA> m_data;
  RawDataA& operator()() { return *m_data; }
  const RawDataA& operator()() const { return *m_data; }
  DataA() : m_data(std::make_shared<RawDataA>()) {}
  explicit DataA(const RawDataA& t_data) { *m_data = t_data; }
  explicit DataA(const std::shared_ptr<RawDataA>& t_data) : m_data(t_data) {}
};

struct DataB {
  std::shared_ptr<RawDataB> m_data;
  RawDataB& operator()() { return *m_data; }
  const RawDataB& operator()() const { return *m_data; }
  DataB() : m_data(std::make_shared<RawDataB>()) {}
  explicit DataB(const RawDataB& t_data) { *m_data = t_data; }
  explicit DataB(const std::shared_ptr<RawDataB>& t_data) : m_data(t_data) {}
};

struct DataAB {
  std::shared_ptr<RawDataA> m_data_a;
  std::shared_ptr<RawDataB> m_data_b;
  DataAB& operator()() { return *this; }
  const DataAB& operator()() const { return *this; }
  DataAB()
      : m_data_a(std::make_shared<RawDataA>()),
        m_data_b(std::make_shared<RawDataB>()) {}
  explicit DataAB(const RawDataA& t_data_a, const RawDataB& t_data_b) {
    *m_data_a = t_data_a;
    *m_data_b = t_data_b;
  }
  explicit DataAB(const std::shared_ptr<RawDataA>& t_data_a,
                  const std::shared_ptr<RawDataB>& t_data_b)
      : m_data_a(t_data_a), m_data_b(t_data_b) {}
  RawDataA& data_a() { return *m_data_a; }
  RawDataB& data_b() { return *m_data_b; }
};

bool updateA(DataA data) {
  data().pos = 2. * data().pos;
  return true;
}

bool updateB(DataB data) {
  data().pos = 0.5 * data().pos;
  return true;
}

bool updateAB(DataAB data) {
  updateA(DataA(data.data_a()));
  updateB(DataB(data.data_b()));
  return true;
}

TEST_CASE("test") {
  DataA a;
  DataB b;
  DataAB ab;
}

TEST_CASE("test 2") {
  std::shared_ptr<RawDataA> a_ptr;
  std::shared_ptr<RawDataB> b_ptr;
  DataAB ab(a_ptr, b_ptr);
}

}  // namespace test

}  // namespace
}  // namespace holon
