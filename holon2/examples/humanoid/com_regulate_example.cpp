/* com_regulate_example - Example of COM regulator
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

#include <iostream>
#include "holon2/corelib/humanoid/com_controller.hpp"

const double T = 10;
const double DT = 0.01;

using holon::ComController;
using holon::Vec3d;

holon::ComControllerParams& getParamsRef(const holon::ComController& ctrl) {
  return const_cast<holon::ComControllerParams&>(ctrl.params());
}

void logging(const holon::ComController& ctrl) {
  Eigen::IOFormat oneliner(Eigen::StreamPrecision, Eigen::DontAlignCols, " ",
                           " ", "", "", "", "");
  std::cout << ctrl.time() << " ";
  std::cout << ctrl.states().com_position.format(oneliner) << " ";
  std::cout << ctrl.states().com_velocity.format(oneliner) << " ";
  std::cout << ctrl.states().zmp_position.format(oneliner) << "\n";
}

int main() {
  ComController ctrl;
  auto& params = getParamsRef(ctrl);
  const Vec3d p0(0.5, -0.5, 0.8);
  const Vec3d pd(0, 0, 1);

  ctrl.reset(p0);
  params.com_position = pd;
  while (ctrl.time() < T) {
    ctrl.update(DT);
    logging(ctrl);
  }
  return 0;
}
