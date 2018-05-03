/* ode_runge_kutta4 - ODE quadrapture: classical Runge-Kutta method
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

#ifndef HOLON_MATH_ODE_RUNGE_KUTTA4_HPP_
#define HOLON_MATH_ODE_RUNGE_KUTTA4_HPP_

#include "holon/corelib/math/ode_solver.hpp"

namespace holon {

template <typename State>
class RungeKutta4 : public OdeSolver<RungeKutta4<State>> {
  using Base = OdeSolver<RungeKutta4<State>>;

 public:
  RungeKutta4() = default;
  virtual ~RungeKutta4() = default;

  template <typename System, typename Time>
  State update_impl(const System& system, const State& x, const Time t,
                    const Time dt);

 private:
  State xm;
  State k[4];
};

template <typename State>
template <typename System, typename Time>
State RungeKutta4<State>::update_impl(const System& system, const State& x,
                                      const Time t, const Time dt) {
  Time dt1, dt2, dt3;
  dt1 = dt * 0.5;
  dt2 = dt / 6;
  dt3 = dt2 * 2;
  k[0] = system(x, t);
  xm = Base::cat(x, dt1, k[0]);
  k[1] = system(xm, t + dt1);
  xm = Base::cat(x, dt1, k[1]);
  k[2] = system(xm, t + dt1);
  xm = Base::cat(x, dt, k[2]);
  k[3] = system(xm, t + dt);

  xm = x;
  xm = Base::cat(xm, dt2, k[0]);
  xm = Base::cat(xm, dt3, k[1]);
  xm = Base::cat(xm, dt3, k[2]);
  xm = Base::cat(xm, dt2, k[3]);
  return xm;
}

}  // namespace holon

#endif  // HOLON_MATH_ODE_RUNGE_KUTTA4_HPP_
