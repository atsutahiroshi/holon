/* ode_runge_kutta4 - ODE quadrapture: classical Runge-Kutta method
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

#include "holon/corelib/math/ode_runge_kutta4.hpp"

#include <array>

#include "catch.hpp"

namespace holon {
namespace {

using State = std::array<double, 1>;
using Time = double;

struct sys {
  State dxdt;
  State operator()(const State&, const Time) const {
    State dxdt;
    dxdt[0] = 1.0;
    return dxdt;
  }
};

TEST_CASE("Check ODE quadrapture with classical Runge-Kutta method",
          "[ode][RungeKutta4]") {
  RungeKutta4<State> solver;
  State x = {{10.0}};
  Time dt = 0.1;
  x = solver.update(sys(), x, 0.0, dt);
  CHECK(x[0] == Approx(10.1));
}

}  // namespace
}  // namespace holon
