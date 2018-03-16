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

}  // namespace
}  // namespace holon
