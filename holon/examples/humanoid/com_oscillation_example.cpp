/* com_regulation_example - example of regulation of COM
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

#include <iostream>
#include "holon/corelib/humanoid/com_ctrl.hpp"

const double T = 10;
const double DT = 0.01;

using holon::Vec3D;
using holon::experimental::ComCtrl;

int main() {
  ComCtrl ctrl;
  auto cmd = ctrl.getCommands();
  Vec3D p0 = {0, 0, 0.42};
  Vec3D pd = {0, 0, 0.42};
  double dist = 0.1;

  ctrl.reset(p0);
  ctrl.states().com_velocity = {0, 0.00001, 0};
  cmd->set_com_position(pd);
  cmd->rho = 1;
  cmd->dist = dist;
  double yzmin = 0;
  double yzmax = 0;
  const double& yz = ctrl.states().zmp_position[1];
  while (ctrl.time() < T) {
    ctrl.update(DT);

    if (yzmax < yz) {
      yzmax = yz;
    }
    if (yzmin > yz) {
      yzmin = yz;
    }

    std::cout << ctrl.time() << " ";
    std::cout << ctrl.states().com_position.data() << " ";
    std::cout << ctrl.states().com_velocity.data() << " ";
    std::cout << ctrl.states().zmp_position.data() << "\n";
  }
  std::cerr << "actual dist = " << (yzmax - yzmin) << "\n";
  return 0;
}
