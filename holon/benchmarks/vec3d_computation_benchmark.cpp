/* vec3d_computation_benchmark - benchmark test for computation of Vec3D class
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

#include "holon/corelib/math/vec3d.hpp"
#include <utility>
#include "hayai.hpp"

namespace holon {
namespace {

class AdditionBenchmark : public ::hayai::Fixture {
 public:
  virtual void SetUp() {
    zVec3DCreate(&a1, 1, 2, 3);
    zVec3DCreate(&a2, 4, 5, 6);
    b1 = {1, 2, 3};
    b2 = {4, 5, 6};
  }
  virtual void TearDown() {}

  zVec3D a1, a2, a;
  zvec3d::Vec3D b1, b2, b;
};

BENCHMARK_F(AdditionBenchmark, zVec3D, 100, 1000) { zVec3DAdd(&a1, &a2, &a); }
BENCHMARK_F(AdditionBenchmark, zvec3d_Vec3D, 100, 1000) { b = b1 + b2; }

class TripleAdditionBenchmark : public ::hayai::Fixture {
 public:
  virtual void SetUp() {
    zVec3DCreate(&a1, 1, 2, 3);
    zVec3DCreate(&a2, 4, 5, 6);
    zVec3DCreate(&a3, 7, 8, 9);
    b1 = {1, 2, 3};
    b2 = {4, 5, 6};
    b3 = {7, 8, 9};
  }
  virtual void TearDown() {}

  zVec3D a1, a2, a3, a;
  zvec3d::Vec3D b1, b2, b3, b;
};

BENCHMARK_F(TripleAdditionBenchmark, zVec3D, 100, 1000) {
  zVec3DAdd(&a1, &a2, &a);
  zVec3DAdd(&a, &a3, &a);
}
BENCHMARK_F(TripleAdditionBenchmark, zvec3d_Vec3D, 100, 1000) {
  b = b1 + b2 + b3;
}

class ConcatenateBenchmark : public ::hayai::Fixture {
 public:
  virtual void SetUp() {
    k = 10;
    zVec3DCreate(&a1, 1, 2, 3);
    zVec3DCreate(&a2, 4, 5, 6);
    b1 = {1, 2, 3};
    b2 = {4, 5, 6};
  }
  virtual void TearDown() {}

  double k;
  zVec3D a1, a2, a;
  zvec3d::Vec3D b1, b2, b;
};

BENCHMARK_F(ConcatenateBenchmark, zVec3D, 100, 1000) {
  zVec3DCat(&a1, k, &a2, &a);
}
BENCHMARK_F(ConcatenateBenchmark, zvec3d_Vec3D, 100, 1000) { b = b1 + k * b2; }

}  // namespace
}  // namespace holon
