/* biped_foot_model_simulator - Simple simulator of foot model dynamics
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

#include "holon2/corelib/humanoid/biped_foot_model/biped_foot_model_simulator.hpp"

#include <array>
#include <memory>
#include "holon2/corelib/humanoid/biped_foot_model.hpp"
#include "holon2/corelib/math/ode_runge_kutta4.hpp"

namespace holon {

class BipedFootModelSimulator::Impl {
  using State = std::array<Vec3d, 2>;
  using Solver = RungeKutta4<State>;
  using System = std::function<State(const State, const double)>;

 public:
  Impl() : Impl(BipedFootModelBuilder().build().data()) {}
  explicit Impl(const Model& t_model) : Impl(t_model.clone().data()) {}
  explicit Impl(const Data& t_data)
      : m_model(t_data), m_initial_position(model().position()), m_solver() {
    setForceDefault();
    setAccelDefault();
    setSystem();
  }

  const Model& model() const { return m_model; }
  Vec3d getInitialPosition() const { return m_initial_position; }

  void setInitialPosition(const Vec3d& t_p0) { m_initial_position = t_p0; }

  Vec3d getAccel(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_accel_functor(p, v, t);
  }

  Vec3d getForce(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_force_functor(p, v, t);
  }

  void setAccel(Functor t_functor) { m_accel_functor = t_functor; }
  void setForce(Functor t_functor) { m_force_functor = t_functor; }
  void setForce(const Vec3d& t_f) { setForce(returnConstVecFunctor(t_f)); }

  void reset() {
    m_model.states().position = getInitialPosition();
    m_model.states().velocity.setZero();
  }

  bool update(const Vec3d& p, const Vec3d& v, const double t, const double dt) {
    auto state = m_solver.update(m_system, State{{p, v}}, t, dt);
    m_model.states().position = state[0];
    m_model.states().velocity = state[1];
    m_model.states().acceleration = getAccel(p, v, t);
    m_model.states().force = getForce(p, v, t);
    return true;
  }

 private:
  Functor returnConstVecFunctor(const Vec3d& v) {
    return [v](const Vec3d&, const Vec3d&, const double) { return v; };
  }

  void setAccelDefault() {
    setAccel([this](const Vec3d& p, const Vec3d& v, const double t) -> Vec3d {
      return getForce(p, v, t) / m_model.mass();
    });
  }

  void setForceDefault() { setForce(returnConstVecFunctor(kVec3dZero)); }

  void setSystem() {
    m_system = [this](const State state, const double t) -> State {
      State dxdt;
      dxdt[0] = state[1];
      dxdt[1] = getAccel(state[0], state[1], t);
      return dxdt;
    };
  }

 private:
  Model m_model;
  Vec3d m_initial_position;
  Functor m_accel_functor;
  Functor m_force_functor;
  Solver m_solver;
  System m_system;
};

BipedFootModelSimulator::BipedFootModelSimulator() : m_impl(new Impl) {}
BipedFootModelSimulator::BipedFootModelSimulator(const Data& t_data)
    : m_impl(new Impl(t_data)) {}
BipedFootModelSimulator::BipedFootModelSimulator(const Model& t_model)
    : m_impl(new Impl(t_model)) {}

BipedFootModelSimulator::BipedFootModelSimulator(BipedFootModelSimulator&&) =
    default;
BipedFootModelSimulator& BipedFootModelSimulator::operator=(
    BipedFootModelSimulator&&) = default;
BipedFootModelSimulator::~BipedFootModelSimulator() = default;

const BipedFootModel& BipedFootModelSimulator::model() const {
  return m_impl->model();
}

Vec3d BipedFootModelSimulator::getInitialPosition() const {
  return m_impl->getInitialPosition();
}

BipedFootModelSimulator& BipedFootModelSimulator::setInitialPosition() {
  return setInitialPosition(model().position());
}

BipedFootModelSimulator& BipedFootModelSimulator::setInitialPosition(
    const Vec3d& t_p0) {
  m_impl->setInitialPosition(t_p0);
  return *this;
}

Vec3d BipedFootModelSimulator::getAccel(const Vec3d& p, const Vec3d& v,
                                        const double t) const {
  return m_impl->getAccel(p, v, t);
}

Vec3d BipedFootModelSimulator::getForce(const Vec3d& p, const Vec3d& v,
                                        const double t) const {
  return m_impl->getForce(p, v, t);
}

BipedFootModelSimulator& BipedFootModelSimulator::setForce(Functor t_functor) {
  m_impl->setForce(t_functor);
  return *this;
}

BipedFootModelSimulator& BipedFootModelSimulator::setForce(const Vec3d& t_f) {
  m_impl->setForce(t_f);
  return *this;
}

BipedFootModelSimulator& BipedFootModelSimulator::reset() {
  m_impl->reset();
  resetTime();
  return *this;
}

BipedFootModelSimulator& BipedFootModelSimulator::reset(
    const Vec3d& t_com_position) {
  setInitialPosition(t_com_position);
  return reset();
}

bool BipedFootModelSimulator::update() { return update(time_step()); }
bool BipedFootModelSimulator::update(double dt) {
  m_impl->update(model().position(), model().velocity(), time(), dt);
  updateTime(dt);
  return true;
}

}  // namespace holon
