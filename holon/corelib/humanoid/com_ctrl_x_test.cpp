/* com_ctrl_x - COM controller along x axis
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

#include "holon/corelib/humanoid/com_ctrl_x.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

TEST_CASE("control poles q1, q2 can be assigned", "[corelib][humanoid]") {
  Fuzzer fuzz;

  SECTION("instantiate with no paramters") {
    ComCtrlX ctrl;

    SECTION("default values of q1 and q2 are 1") {
      CHECK(ctrl.q1() == 1.0);
      CHECK(ctrl.q2() == 1.0);
    }

    SECTION("q1, q2 can be assigned") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.set_q1(q1);
        ctrl.set_q2(q2);
        CHECK(ctrl.q1() == q1);
        CHECK(ctrl.q2() == q2);
      }
    }

    SECTION("q1, q2 can be assigned with chaining method") {
      for (auto i = 0; i < 3; ++i) {
        double q1 = fuzz.get();
        double q2 = fuzz.get();
        ctrl.set_q1(q1).set_q2(q2);
        CHECK(ctrl.q1() == q1);
        CHECK(ctrl.q2() == q2);
      }
    }
  }

  SECTION("instantiate with paramters") {
    for (auto i = 0; i < 3; ++i) {
      double q1 = fuzz.get();
      double q2 = fuzz.get();

      ComCtrlX ctrl(q1, q2);
      CHECK(ctrl.q1() == q1);
      CHECK(ctrl.q2() == q2);
    }
  }

  SECTION("instantiate with paramters then set arbitrary value") {
    for (auto i = 0; i < 3; ++i) {
      double q1 = fuzz.get();
      double q2 = fuzz.get();

      ComCtrlX ctrl(q1, q2);
      CHECK(ctrl.q1() == q1);
      CHECK(ctrl.q2() == q2);

      q1 = fuzz.get();
      q2 = fuzz.get();
      ctrl.set_q1(q1);
      ctrl.set_q2(q2);
      CHECK(ctrl.q1() == q1);
      CHECK(ctrl.q2() == q2);
    }
  }
}

}  // namespace
}  // namespace holon
