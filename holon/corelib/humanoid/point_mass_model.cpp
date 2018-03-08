/* point_mass_model - Point mass model
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

#include "holon/corelib/humanoid/point_mass_model.hpp"

#include "holon/corelib/common/zip.hpp"

namespace holon {

namespace internal {

template <typename State, typename Time>
State concatenate(const State& x, const Time dt, const State& dxdt) {
  State ret;
  for (auto&& i : zip_ptr(x, dxdt, ret)) {
    *std::get<2>(i) = *std::get<0>(i) + *std::get<1>(i) * dt;
  }
  return ret;
}

template <typename System, typename State, typename Time>
State update_rk4(const System& system, const State& x, const Time t,
                 const Time dt) {
  State k1, k2, k3, k4, xm;
  Time dt1, dt2, dt3;

  dt1 = dt * 0.5;
  dt2 = dt / 6;
  dt3 = dt2 * 2;

  k1 = system(x, t);
  xm = concatenate(x, dt1, k1);
  k2 = system(xm, t + dt1);
  xm = concatenate(x, dt1, k2);
  k3 = system(xm, t + dt1);
  xm = concatenate(x, dt, k3);
  k4 = system(xm, t + dt);

  xm = x;
  xm = concatenate(xm, dt2, k1);
  xm = concatenate(xm, dt3, k2);
  xm = concatenate(xm, dt3, k3);
  xm = concatenate(xm, dt2, k4);
  return xm;
}

template <typename System, typename State, typename Time>
State update_euler(const System& system, const State& x, const Time t,
                   const Time dt) {
  return concatenate(x, dt, system(x, t));
}

}  // namespace internal

}  // namespace holon
