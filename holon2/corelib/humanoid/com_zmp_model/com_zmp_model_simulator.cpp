/* com_zmp_model_simulator - Simple simulator of the dynamics of COM-ZMP model
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

#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_simulator.hpp"

#include <memory>
#include "holon2/corelib/humanoid/com_zmp_model.hpp"
#include "holon2/corelib/humanoid/const_defs.hpp"

namespace holon {

class ComZmpModelSimulator::Impl {
 public:
  Impl() : m_model(ComZmpModelBuilder().build()) {}
  // explicit Impl(Data t_data) {}
  // explicit Impl(const Model& t_model) {}

  const Model& model() const { return m_model; }

 private:
  Model m_model;
};

ComZmpModelSimulator::ComZmpModelSimulator() : m_impl(new Impl) {}

ComZmpModelSimulator::ComZmpModelSimulator(ComZmpModelSimulator&&) = default;
ComZmpModelSimulator& ComZmpModelSimulator::operator=(ComZmpModelSimulator&&) =
    default;
ComZmpModelSimulator::~ComZmpModelSimulator() = default;

const ComZmpModel& ComZmpModelSimulator::model() const {
  return m_impl->model();
}

void ComZmpModelSimulator::reset() { resetTime(); }
bool ComZmpModelSimulator::update() { return update(time_step()); }
bool ComZmpModelSimulator::update(double dt) {
  updateTime(dt);
  return true;
}

}  // namespace holon
