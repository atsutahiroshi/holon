/* com_regulation_with_disturbance_example - example of COM regulator with
 * disturbance
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

void apply_external_force(ComZmpModel* model, double t) {
  Vec3D force1 = {1, -1, 0};
  Vec3D force2 = {-1.5, 1.5, 0};

  if (t > 4 && t < 4.1) {
    model->set_external_force(force1);
  } else if (t > 6 && t < 6.1) {
    model->set_external_force(force2);
  } else {
    model->clear_external_force();
  }
}

int main() {
  ComZmpModel model;
  model.reset(Vec3D(0.1, -0.1, 1));

  ComCtrl ctrl(model);
  auto cmd = ctrl.getCommands();
  Vec3D cmd_com_pos = {0, 0, 1};

  double t = 0;
  while (t < T) {
    // feedback
    ctrl.feedback(model);

    // update controller
    cmd->set_com_position(cmd_com_pos);
    ctrl.update(DT);

    // update simulator
    apply_external_force(&model, t);
    model.inputZmpPos(ctrl.outputs().zmp_position);
    model.update(DT);

    // logging
    std::cout << t << " ";
    std::cout << model.data().com_position.data() << " ";
    std::cout << model.data().com_velocity.data() << " ";
    std::cout << model.data().zmp_position.data() << "\n";
    t += DT;
  }
  return 0;
}
