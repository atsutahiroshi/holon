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
  Impl()
      : m_model(ComZmpModelBuilder().build()),
        m_ctrl_input_type(CtrlInputType::kNotDetermined),
        m_initial_com_position(model().com_position()) {}
  explicit Impl(Data t_data)
      : m_model(t_data),
        m_ctrl_input_type(CtrlInputType::kNotDetermined),
        m_initial_com_position(model().com_position()) {}
  explicit Impl(const Model& t_model)
      : m_model(t_model.clone()),
        m_ctrl_input_type(CtrlInputType::kNotDetermined),
        m_initial_com_position(model().com_position()) {}

  const Model& model() const { return m_model; }
  CtrlInputType getCtrlInputType() const { return m_ctrl_input_type; }
  Vec3d getInitialComPosition() const { return m_initial_com_position; }

  void setCtrlInputType(CtrlInputType t_type) { m_ctrl_input_type = t_type; }
  void setInitialComPosition(const Vec3d& t_p0) {
    m_initial_com_position = t_p0;
  }

 private:
  Model m_model;
  CtrlInputType m_ctrl_input_type;
  Vec3d m_initial_com_position;
};

ComZmpModelSimulator::ComZmpModelSimulator() : m_impl(new Impl) {}
ComZmpModelSimulator::ComZmpModelSimulator(Data t_data)
    : m_impl(new Impl(t_data)) {}
ComZmpModelSimulator::ComZmpModelSimulator(const Model& t_model)
    : m_impl(new Impl(t_model)) {}

ComZmpModelSimulator::ComZmpModelSimulator(ComZmpModelSimulator&&) = default;
ComZmpModelSimulator& ComZmpModelSimulator::operator=(ComZmpModelSimulator&&) =
    default;
ComZmpModelSimulator::~ComZmpModelSimulator() = default;

const ComZmpModel& ComZmpModelSimulator::model() const {
  return m_impl->model();
}

ComZmpModelSimulator::CtrlInputType ComZmpModelSimulator::getCtrlInputType()
    const {
  return m_impl->getCtrlInputType();
}

Vec3d ComZmpModelSimulator::getInitialComPosition() const {
  return m_impl->getInitialComPosition();
}

ComZmpModelSimulator& ComZmpModelSimulator::setCtrlInputType(
    CtrlInputType t_type) {
  m_impl->setCtrlInputType(t_type);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::setInitialComPosition(
    const Vec3d& t_p0) {
  m_impl->setInitialComPosition(t_p0);
  return *this;
}

void ComZmpModelSimulator::reset() { resetTime(); }
bool ComZmpModelSimulator::update() { return update(time_step()); }
bool ComZmpModelSimulator::update(double dt) {
  updateTime(dt);
  return true;
}

}  // namespace holon
