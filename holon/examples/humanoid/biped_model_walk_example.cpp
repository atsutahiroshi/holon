/* biped_model_walk_example - example of bipedal walking control
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
#include "holon/corelib/humanoid/biped_ctrl.hpp"

const double T = 10;
const double DT = 0.01;

using holon::Vec3D;
using holon::BipedCtrl;

int main() {
  BipedCtrl ctrl;
  // auto cmd = ctrl.get_commands_handle();
  // Vec3D p0{0, 0, 0.42};

  // ctrl.reset(p0, dist);
  ctrl.states().com_velocity = {0, 0.00001, 0};
  while (ctrl.time() < T) {
    ctrl.update(DT);
    // logging
    std::cout << ctrl.time() << " ";
    std::cout << ctrl.states().com_position.data() << " ";
    std::cout << ctrl.states().com_velocity.data() << " ";
    std::cout << ctrl.states().zmp_position.data() << " ";
    std::cout << "\n";
  }
  return 0;
}
