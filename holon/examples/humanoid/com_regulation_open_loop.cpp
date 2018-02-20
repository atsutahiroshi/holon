/* com_regulation_open_loop - example of regulation of COM
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

int main() {
  holon::ComCtrl ctrl;
  auto cmd = ctrl.getUserCommands();
  holon::Vec3D initial_com_pos = {0.1, -0.1, 1};
  holon::Vec3D cmd_com_pos = {0, 0, 1};
  double t = 0;

  ctrl.reset(initial_com_pos);
  while (t < T) {
    cmd->com_position = cmd_com_pos;
    ctrl.update(DT);
    std::cout << t << " ";
    std::cout << ctrl.model().data()->com_position.data() << " ";
    std::cout << ctrl.model().data()->com_velocity.data() << " ";
    std::cout << ctrl.model().data()->zmp_position.data() << "\n";
    t += ctrl.time_step();
  }
  return 0;
}
