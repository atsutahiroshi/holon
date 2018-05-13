/* biped_foot_controller - Biped foot controller class
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

#include "holon2/corelib/humanoid/biped_foot_controller.hpp"

namespace holon {

const Array3d BipedFootController::default_stiffness = {{1, 1, 1}};
const Array3d BipedFootController::default_damping = {{1, 1, 1}};
const double BipedFootController::default_max_height = 0;

BipedFootController::BipedFootController() : BipedFootController(Data()) {}
BipedFootController::BipedFootController(const Data& t_data)
    : m_data(t_data), m_sim(m_data.subdata<0, 1>()) {
  BipedFootModelBuilder().build(m_data.subdata<0, 1>());
  setDefaultParameters();
  setupSimulator();
}

BipedFootController::BipedFootController(const Model& t_model)
    : BipedFootController() {
  copyModelData(t_model);
  reset(states().position);
}

void BipedFootController::setDefaultParameters() {
  params().position = states().position;
  params().velocity = kVec3dZero;
  params().stiffness = default_stiffness;
  params().damping = default_damping;
  params().max_height = default_max_height;
}

void BipedFootController::setupSimulator() {}

void BipedFootController::copyModelData(const Model& t_model) {
  m_data.copy(t_model.data(), IndexSeq<0, 1>(), IndexSeq<0, 1>());
}

BipedFootController& BipedFootController::setTimeStep(double t_time_step) {
  m_sim.setTimeStep(t_time_step);
  return *this;
}

BipedFootController& BipedFootController::reset() {
  return reset(getInitialPosition());
}

BipedFootController& BipedFootController::reset(const Vec3d& t_position) {
  m_sim.reset(t_position);
  params().position = t_position;
  return *this;
}

// BipedFootController& BipedFootController::feedback(const Model& t_model) {}
// BipedFootController& BipedFootController::feedback(
//     const BipedFootModelData& t_model_data) {}

// bool BipedFootController::update() {}
// bool BipedFootController::update(double t_time_step) {}

}  // namespace holon
