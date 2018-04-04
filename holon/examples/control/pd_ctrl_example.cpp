/* pd_ctrl_example - Example of PD control
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

#include "holon/corelib/control/pd_ctrl.hpp"
#include <cmath>
#include <iostream>

using holon::Vec3D;
using holon::PdCtrl;

const double T = 10;
const double DT = 0.01;
const double omega = 2;
const double amp = 1.0;
const double phase = 0.0;

double traj(double t) { return amp * std::sin(omega * t - phase); }

double traj_vel(double t) { return amp * omega * std::cos(omega * t - phase); }

void log(const PdCtrl<double>& ctrl) {
  std::cout << ctrl.time() << " ";
  std::cout << ctrl.refs().position << " ";
  std::cout << ctrl.refs().velocity << " ";
  std::cout << ctrl.states().position << " ";
  std::cout << ctrl.states().velocity << "\n";
}

int main() {
  PdCtrl<double> ctrl;
  ctrl.refs().stiffness = 100;
  ctrl.refs().damping = 10;

  log(ctrl);
  while (ctrl.time() < T) {
    ctrl.refs().position = traj(ctrl.time());
    ctrl.refs().velocity = traj_vel(ctrl.time());
    ctrl.update(DT);
    log(ctrl);
  }
  return 0;
}
