/* com_regulation_vertical_example - COM regulation along vertical direction
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
using holon::ComZmpModel;
using holon::ComCtrl;

Vec3D force = {0, 0, -1};
auto external_force = [](const Vec3D&, const Vec3D&, const double t) {
  if (t > 3 && t < 3.1) {
    return force;
  } else {
    return holon::kVec3DZero;
  }
};

int main() {
  Vec3D p0 = {0, 0, 0.4};
  ComZmpModel model;
  model.reset(p0);

  ComCtrl ctrl(model);
  auto cmd = ctrl.getCommands();
  Vec3D pd1 = {0, 0, 0.42};
  Vec3D pd2 = {0, 0, 0.44};
  cmd->set_com_position(pd1);

  // model.setZmpPositionCallback(ctrl.getZmpPositionUpdater());
  model.setExternalForceCallback(external_force);

  while (model.time() < T) {
    // feedback
    ctrl.feedback(model);

    // update controller
    if (model.time() < 6) {
      cmd->set_com_position(pd1);
    } else {
      cmd->set_com_position(pd2);
    }
    ctrl.update(DT);

    // update simulator
    model.setReactionForce(ctrl.outputs().reaction_force);
    model.update(DT);

    // logging
    std::cout << model.time() << " ";
    std::cout << ctrl.commands().xd.value() << " ";
    std::cout << ctrl.commands().yd.value() << " ";
    std::cout << ctrl.commands().zd.value() << " ";
    std::cout << model.states().com_position.data() << " ";
    std::cout << model.states().com_velocity.data() << " ";
    std::cout << model.states().zmp_position.data() << "\n";
  }
  return 0;
}
