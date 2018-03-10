/* ode_solver - Ordinary differential equation quadrapture
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

#ifndef HOLONE_MATH_ODE_SOLVER_HPP_
#define HOLONE_MATH_ODE_SOLVER_HPP_

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

 protected:
  Solver& solver() { return *static_cast<Solver*>(this); }
  const Solver& solver() const { return *static_cast<Solver*>(this); }

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
};

}  // namespace holon

#endif  // HOLONE_MATH_ODE_SOLVER_HPP_
