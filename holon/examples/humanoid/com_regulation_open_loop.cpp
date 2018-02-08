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
  zVec3D initial_com_pos = {{0.1, -0.1, 1}};
  zVec3D cmd_com_pos = {{0, 0, 1}};
  double t = 0;

  ctrl.model().reset_com_position(&initial_com_pos);
  while (t < T) {
    ctrl.set_cmd_com_position(&cmd_com_pos);
    ctrl.update(DT);
    std::cout << t << " ";
    std::cout << zVec3DElem(ctrl.model().com_position(), zX) << " "
              << zVec3DElem(ctrl.model().com_position(), zY) << " "
              << zVec3DElem(ctrl.model().com_position(), zZ) << " "
              << zVec3DElem(ctrl.model().com_velocity(), zX) << " "
              << zVec3DElem(ctrl.model().com_velocity(), zY) << " "
              << zVec3DElem(ctrl.model().com_velocity(), zZ) << " "
              << zVec3DElem(ctrl.des_zmp_position(), zX) << " "
              << zVec3DElem(ctrl.des_zmp_position(), zY) << " "
              << zVec3DElem(ctrl.des_zmp_position(), zZ) << " "
              << "\n";
    t += ctrl.time_step();
  }
  return 0;
}
