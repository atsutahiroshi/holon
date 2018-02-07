/* com_ctrl_benchmark - benchmark test for ComCtrl class
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

#include "holon/corelib/humanoid/com_ctrl.hpp"

#include "hayai.hpp"

namespace holon {
namespace {

class ComCtrlBenchmark : public ::hayai::Fixture {
 public:
  virtual void SetUp() {
    zVec3DCreate(&ref_com_pos, 0, 0, 1);
    zVec3DCreate(&com_pos, 0, 0, 1);
    zVec3DCreate(&com_vel, 0, 0, 0);
  }
  virtual void TearDown() {}

  ComCtrl ctrl;
  zVec3D ref_com_pos;
  zVec3D com_pos;
  zVec3D com_vel;
};

BENCHMARK_F(ComCtrlBenchmark, GetQ1, 100, 1000) {
  double q1 = ctrl.x().q1();
  (void)q1;
}

BENCHMARK_F(ComCtrlBenchmark, SetQ1, 100, 1000) { ctrl.x().set_q1(1); }

BENCHMARK_F(ComCtrlBenchmark, ComputeDesiredZmpPosition, 100, 1000) {
  zVec3D desired_zmp_pos;
  ctrl.ComputeDesiredZmpPosition(&ref_com_pos, &com_pos, &com_vel,
                                 &desired_zmp_pos);
}

}  // namespace
}  // namespace holon
