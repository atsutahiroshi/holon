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
#include <type_traits>
#include "holon2/corelib/common/random.hpp"
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

TEST_CASE("dataset: raw data is kept as a smart pointer in Dataset",
          "[Dataset]") {
  TestDataset1 data;
  auto ptr1 = data.getRawDataPtr<0>();
  auto ptr2 = data.getRawDataPtr<1>();
  CHECK(std::is_same<decltype(ptr1), std::shared_ptr<TestRawData1>>::value);
  CHECK(std::is_same<decltype(ptr2), std::shared_ptr<TestRawData2>>::value);
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

}  // namespace
}  // namespace holon
