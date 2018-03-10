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

#ifndef HOLON_MATH_ODE_EULER_HPP_
#define HOLON_MATH_ODE_EULER_HPP_

#include "holon/corelib/common/zip.hpp"

namespace holon {

template <typename Solver>
class OdeSolver {
 public:
  OdeSolver() = default;
  virtual ~OdeSolver() = default;

  template <typename System, typename State, typename Time>
  State update(const System& system, const State& x, const Time t,
               const Time dt) {
    return this->solver().update_impl(system, x, t, dt);
  }

 private:
  Solver& solver() { return *static_cast<Solver*>(this); }
};

namespace operations {

template <typename State, typename Time>
State cat(const State& state, const Time dt, const State& deriv) {
  State state_out;
  auto x = state.begin();
  auto dxdt = deriv.begin();
  auto x_out = state_out.begin();
  for (; x_out != state_out.end(); ++x, ++dxdt, ++x_out) {
    *x_out = *x + *dxdt * dt;
  }
  return state_out;
}

}  // namespace operations

template <typename State>
class RungeKutta4 : public OdeSolver<RungeKutta4<State>> {};

template <typename State>
class Euler : public OdeSolver<Euler<State>> {
 public:
  Euler() = default;
  virtual ~Euler() = default;

  template <typename System, typename Time>
  State update_impl(const System& system, const State& x, const Time t,
                    const Time dt) {
    return operations::cat(x, dt, system(x, t));
  }
};

}  // namespace holon

#endif  // HOLON_MATH_ODE_EULER_HPP_
