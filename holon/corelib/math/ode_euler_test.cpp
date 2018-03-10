/* ode_euler - ordinary differential equation quadrature: Euler method
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

#include "holon/corelib/math/ode_euler.hpp"

#include <array>
#include <iostream>

#include "catch.hpp"

namespace holon {
namespace {

using State = std::array<double, 2>;
using Time = double;

struct sys {
  State operator()(const State& x, const Time t) const {
    State dxdt;
    dxdt[0] = x[0] + 2.0 * x[1];
    dxdt[1] = x[1];
    return dxdt;
  }
};

TEST_CASE("Check ODE quadrapture with Euler method", "[ode][Euler]") {
  Euler<State> solver;
  State x = {{0, 1}};
  double dt = 0.001;
  x = solver.update(sys(), x, 0.0, dt);
  CHECK(x[0] == Approx(0.0 + (0.0 + 2.0 * 1.0) * dt));
  CHECK(x[1] == Approx(1.0 + (1.0) * dt));
}

}  // namespace
}  // namespace holon
