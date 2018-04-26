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
#include "holon2/corelib/math/vec.hpp"

#include "third_party/catch/catch.hpp"

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
using TestDataset1 = Dataset<TestRawData1>;

TestDataset1 TestDataset1Factory() { return TestDataset1(); }

TEST_CASE("dataset: access to raw data", "[Dataset]") {
  TestDataset1 data;
  data.get().a = 1;
  CHECK(data.get().a == 1);
}
