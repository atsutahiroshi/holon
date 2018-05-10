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

#include <array>
#include <memory>
#include "holon2/corelib/humanoid/com_zmp_model.hpp"
#include "holon2/corelib/humanoid/const_defs.hpp"
#include "holon2/corelib/math/ode_runge_kutta4.hpp"

namespace holon {

namespace cz = com_zmp_model_formula;

class ComZmpModelSimulator::Impl {
  using State = std::array<Vec3d, 2>;
  using Solver = RungeKutta4<State>;
  using System = std::function<State(const State, const double)>;

 public:
  Impl() : Impl(ComZmpModelBuilder().build().data()) {}
  explicit Impl(const Model& t_model) : Impl(t_model.clone().data()) {}
  explicit Impl(Data t_data)
      : m_model(t_data),
        m_input_type(InputType::kNotDetermined),
        m_initial_com_position(model().com_position()),
        m_solver() {
    setExtForceDefault();
    setContactForceDefault();
    setZmpPositionDefault();
    setComAccelDefault();
    setSystem();
  }

  const Model& model() const { return m_model; }
  InputType getInputType() const { return m_input_type; }
  Vec3d getInitialComPosition() const { return m_initial_com_position; }

  bool isZmpPosAsInput() const {
    return getInputType() == InputType::kZmpPosition;
  }
  bool isContactForceAsInput() const {
    return getInputType() == InputType::kContactForce;
  }

  void setInputType(InputType t_type) {
    m_input_type = t_type;
    switch (t_type) {
      case InputType::kContactForce:
        setComAccelWhenInputTypeIsContactForce();
        break;
      case InputType::kZmpPosition:
        setComAccelWhenInputTypeIsZmpPos();
        break;
      default:
        setComAccelDefault();
        break;
    }
  }

  void setInitialComPosition(const Vec3d& t_p0) {
    m_initial_com_position = t_p0;
  }

  Vec3d getComAccel(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_com_accel_functor(p, v, t);
  }
  Vec3d getZmpPosition(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_zmp_position_functor(p, v, t);
  }
  Vec3d getContactForce(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_contact_force_functor(p, v, t);
  }
  Vec3d getExtForce(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_ext_force_functor(p, v, t);
  }

  void setComAccel(Functor t_functor) { m_com_accel_functor = t_functor; }
  void setZmpPosition(Functor t_functor) { m_zmp_position_functor = t_functor; }
  void setZmpPosition(const Vec3d& t_pz) {
    setZmpPosition(returnConstVecFunctor(t_pz));
  }
  void setContactForce(Functor t_functor) {
    m_contact_force_functor = t_functor;
  }
  void setContactForce(const Vec3d& t_f) {
    setContactForce(returnConstVecFunctor(t_f));
  }
  void setExtForce(Functor t_functor) { m_ext_force_functor = t_functor; }
  void setExtForce(const Vec3d& t_ef) {
    setExtForce(returnConstVecFunctor(t_ef));
  }

  void reset() {
    m_model.states().com_position = getInitialComPosition();
    m_model.states().com_velocity.setZero();
  }

  bool update(const Vec3d& p, const Vec3d& v, const double t, const double dt) {
    auto state = m_solver.update(m_system, State{{p, v}}, t, dt);
    m_model.states().com_position = state[0];
    m_model.states().com_velocity = state[1];
    m_model.states().com_acceleration = getComAccel(p, v, t);
    if (isZmpPosAsInput()) {
      m_model.states().zmp_position = getZmpPosition(p, v, t);
      double fz = getContactForce(p, v, t).z();
      m_model.states().contact_force =
          cz::contactForce(p, m_model.zmp_position(), fz);
    } else if (isContactForceAsInput()) {
      m_model.states().contact_force = getContactForce(p, v, t);
    }
    m_model.states().external_force = getExtForce(p, v, t);
    m_model.states().reaction_force =
        m_model.contact_force() + m_model.external_force();
    return true;
  }

 private:
  Functor returnConstVecFunctor(const Vec3d& v) {
    return [v](const Vec3d&, const Vec3d&, const double) { return v; };
  }

  void setComAccelDefault() { setComAccel(returnConstVecFunctor(kVec3dZero)); }
  void setComAccelWhenInputTypeIsContactForce() {
    setComAccel([this](const Vec3d& p, const Vec3d& v, const double t) {
      return cz::comAccel(getContactForce(p, v, t), m_model.mass(),
                          getExtForce(p, v, t));
    });
  }
  void setComAccelWhenInputTypeIsZmpPos() {
    setComAccel([this](const Vec3d& p, const Vec3d& v, const double t) {
      return cz::comAccel(p, getZmpPosition(p, v, t), getContactForce(p, v, t),
                          m_model.mass(), getExtForce(p, v, t));
    });
  }
  void setZmpPositionDefault() {
    setZmpPosition([this](const Vec3d&, const Vec3d&, const double) {
      Vec3d ret = m_model.com_position();
      ret.z() = m_model.vhp();
      return ret;
    });
  }
  void setContactForceDefault() {
    setContactForce([this](const Vec3d&, const Vec3d&, const double) {
      return Vec3d(0, 0, m_model.mass() * kGravAccel);
    });
  }
  void setExtForceDefault() { setExtForce(returnConstVecFunctor(kVec3dZero)); }
  void setSystem() {
    m_system = [this](const State state, const double t) -> State {
      State dxdt;
      dxdt[0] = state[1];
      dxdt[1] = getComAccel(state[0], state[1], t);
      return dxdt;
    };
  }

 private:
  Model m_model;
  InputType m_input_type;
  Vec3d m_initial_com_position;
  Functor m_com_accel_functor;
  Functor m_zmp_position_functor;
  Functor m_contact_force_functor;
  Functor m_ext_force_functor;
  Solver m_solver;
  System m_system;
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

ComZmpModelSimulator::InputType ComZmpModelSimulator::getInputType() const {
  return m_impl->getInputType();
}

Vec3d ComZmpModelSimulator::getInitialComPosition() const {
  return m_impl->getInitialComPosition();
}

bool ComZmpModelSimulator::isZmpPosAsInput() const {
  return m_impl->isZmpPosAsInput();
}
bool ComZmpModelSimulator::isContactForceAsInput() const {
  return m_impl->isContactForceAsInput();
}

ComZmpModelSimulator& ComZmpModelSimulator::setInputType(InputType t_type) {
  m_impl->setInputType(t_type);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::setZmpPosAsInput() {
  return setInputType(InputType::kZmpPosition);
}

ComZmpModelSimulator& ComZmpModelSimulator::setContactForceAsInput() {
  return setInputType(InputType::kContactForce);
}

ComZmpModelSimulator& ComZmpModelSimulator::setInitialComPosition() {
  return setInitialComPosition(model().com_position());
}

ComZmpModelSimulator& ComZmpModelSimulator::setInitialComPosition(
    const Vec3d& t_p0) {
  m_impl->setInitialComPosition(t_p0);
  return *this;
}

Vec3d ComZmpModelSimulator::getComAccel(const Vec3d& p, const Vec3d& v,
                                        const double t) const {
  return m_impl->getComAccel(p, v, t);
}

Vec3d ComZmpModelSimulator::getZmpPosition(const Vec3d& p, const Vec3d& v,
                                           const double t) const {
  return m_impl->getZmpPosition(p, v, t);
}

Vec3d ComZmpModelSimulator::getContactForce(const Vec3d& p, const Vec3d& v,
                                            const double t) const {
  return m_impl->getContactForce(p, v, t);
}

Vec3d ComZmpModelSimulator::getExtForce(const Vec3d& p, const Vec3d& v,
                                        const double t) const {
  return m_impl->getExtForce(p, v, t);
}

ComZmpModelSimulator& ComZmpModelSimulator::setZmpPosition(Functor t_functor) {
  m_impl->setZmpPosition(t_functor);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::setZmpPosition(const Vec3d& t_pz) {
  m_impl->setZmpPosition(t_pz);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::setContactForce(Functor t_functor) {
  m_impl->setContactForce(t_functor);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::setContactForce(const Vec3d& t_f) {
  m_impl->setContactForce(t_f);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::setExtForce(Functor t_functor) {
  m_impl->setExtForce(t_functor);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::setExtForce(const Vec3d& t_ef) {
  m_impl->setExtForce(t_ef);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::clearExtForce() {
  m_impl->setExtForce(kVec3dZero);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::reset() {
  m_impl->reset();
  resetTime();
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::reset(const Vec3d& t_com_position) {
  setInitialComPosition(t_com_position);
  return reset();
}

bool ComZmpModelSimulator::update() { return update(time_step()); }

bool ComZmpModelSimulator::update(double dt) {
  m_impl->update(model().com_position(), model().com_velocity(), time(), dt);
  updateTime(dt);
  return true;
}

}  // namespace holon
