/* ode_euler - ordinary differential equation quadrature: Euler method
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

#ifndef HOLON_MATH_ODE_EULER_HPP_
#define HOLON_MATH_ODE_EULER_HPP_

#include "holon/corelib/math/ode_solver.hpp"

namespace holon {

template <typename State>
class Euler : public OdeSolver<Euler<State>> {
  using Base = OdeSolver<Euler<State>>;

 public:
  Euler() = default;
  virtual ~Euler() = default;

  template <typename System, typename Time>
  State update_impl(const System& system, const State& x, const Time t,
                    const Time dt) {
    return Base::cat(x, dt, system(x, t));
  }
};

}  // namespace holon

#endif  // HOLON_MATH_ODE_EULER_HPP_
