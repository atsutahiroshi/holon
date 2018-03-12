/* ode_euler_example - An example for Euler method
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

using state_type = std::array<double, 2>;

// unit circle function
state_type dp(const state_type& p, const double /* t */) {
  state_type v;
  v[0] = -p[1];
  v[1] = p[0];
  return v;
}

void print(const state_type x) { std::cout << x[0] << " " << x[1] << "\n"; }

const double DT = 0.1;
const double T = 10;
int main() {
  holon::Euler<state_type> solver;
  state_type x{{1, 0}};
  print(x);
  for (double t = 0; t < T; t += DT) {
    x = solver.update(dp, x, t, DT);
    print(x);
  }
  return 0;
}
