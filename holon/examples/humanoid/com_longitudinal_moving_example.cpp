/* com_longitudinal_moving_example - example of longitudinal moving
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
  cmd->zd = 0.42;
  // cmd->rho = 1;
  cmd->dist = dist;
  while (ctrl.time() < T) {
    if (ctrl.time() < 5) {
      cmd->vxd = 0.1;
    } else {
      cmd->vxd = 0;
      // cmd->rho = 0;
    }
    ctrl.update(DT);

    std::cout << ctrl.time() << " ";
    std::cout << ctrl.commands().vxd.value_or(0) << " ";
    std::cout << ctrl.commands().vyd.value_or(0) << " ";
    std::cout << ctrl.states().com_position.data() << " ";
    std::cout << ctrl.states().com_velocity.data() << " ";
    std::cout << ctrl.states().zmp_position.data() << "\n";
  }
  return 0;
}
