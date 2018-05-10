/* com_controller - COM controller class
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

#include "holon2/corelib/humanoid/com_controller.hpp"

namespace holon {

ComController::ComController() : ComController(Data()) {}
ComController::ComController(Data t_data)
    : m_data(t_data), m_sim(m_data.subdata<0, 1>()) {}
ComController::ComController(const Model& t_model) : ComController() {
  copyModelData(t_model);
}

void ComController::copyModelData(const Model& t_model) {
  m_data.copy(t_model.data(), IndexSeq<0, 1>(), IndexSeq<0, 1>());
}

ComController& ComController::setTimeStep(double t_time_step) {
  m_sim.setTimeStep(t_time_step);
  return *this;
}

ComController& ComController::reset() {
  m_sim.reset();
  return *this;
}
// ComController& ComController::reset(const Vec3d& t_com_position);

// ComController& ComController::feedback(const Model& t_model);
// ComController& ComController::feedback(ComZmpModelData t_model_data);
// ComController& ComController::feedback(const Vec3d& t_com_position,
//                                        const Vec3d& t_com_velocity);

bool ComController::update() {
  m_sim.update();
  return true;
}

bool ComController::update(double t_time_step) {
  m_sim.update(t_time_step);
  return true;
}

// ComController::Functor ComController::getReactForceFunctor() const;
// ComController::Functor ComController::getZmpPositionFunctor() const;

}  // namespace holon
