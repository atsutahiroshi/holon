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
  Impl() : Impl(ComZmpModelBuilder().build().data()) {}
  explicit Impl(const Model& t_model) : Impl(t_model.clone().data()) {}
  explicit Impl(Data t_data)
      : m_model(t_data),
        m_input_type(InputType::kNotDetermined),
        m_initial_com_position(model().com_position()) {
    setDefaultExtForce();
    setDefaultReactForce();
    setDefaultZmpPosition();
    setDefaultComAccel();
  }

  const Model& model() const { return m_model; }
  InputType getInputType() const { return m_input_type; }
  Vec3d getInitialComPosition() const { return m_initial_com_position; }

  void setInputType(InputType t_type) { m_input_type = t_type; }
  void setInitialComPosition(const Vec3d& t_p0) {
    m_initial_com_position = t_p0;
  }

  Vec3d getComAccel(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_com_accel_functor(p, v, t);
  }
  Vec3d getZmpPosition(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_zmp_position_functor(p, v, t);
  }
  Vec3d getReactForce(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_react_force_functor(p, v, t);
  }
  Vec3d getExtForce(const Vec3d& p, const Vec3d& v, const double t) const {
    return m_ext_force_functor(p, v, t);
  }

  void setComAccel(Functor t_functor) { m_com_accel_functor = t_functor; }
  void setZmpPosition(Functor t_functor) { m_zmp_position_functor = t_functor; }
  void setZmpPosition(const Vec3d& t_pz) {
    setZmpPosition(returnConstVecFunctor(t_pz));
  }
  void setReactForce(Functor t_functor) { m_react_force_functor = t_functor; }
  void setReactForce(const Vec3d& t_f) {
    setReactForce(returnConstVecFunctor(t_f));
  }
  void setExtForce(Functor t_functor) { m_ext_force_functor = t_functor; }
  void setExtForce(const Vec3d& t_ef) {
    setExtForce(returnConstVecFunctor(t_ef));
  }

 private:
  Functor returnConstVecFunctor(const Vec3d& v) {
    return [v](const Vec3d&, const Vec3d&, const double) { return v; };
  }

  void setDefaultComAccel() { setComAccel(returnConstVecFunctor(kVec3dZero)); }
  void setDefaultZmpPosition() {
    setZmpPosition([this](const Vec3d&, const Vec3d&, const double) {
      Vec3d ret = m_model.com_position();
      ret.z() = m_model.vhp();
      return ret;
    });
  }
  void setDefaultReactForce() {
    setReactForce([this](const Vec3d&, const Vec3d&, const double) {
      return Vec3d(0, 0, m_model.mass() * kGravAccel);
    });
  }
  void setDefaultExtForce() { setExtForce(returnConstVecFunctor(kVec3dZero)); }

 private:
  Model m_model;
  InputType m_input_type;
  Vec3d m_initial_com_position;
  Functor m_com_accel_functor;
  Functor m_zmp_position_functor;
  Functor m_react_force_functor;
  Functor m_ext_force_functor;
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

ComZmpModelSimulator& ComZmpModelSimulator::setInputType(InputType t_type) {
  m_impl->setInputType(t_type);
  return *this;
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

Vec3d ComZmpModelSimulator::getReactForce(const Vec3d& p, const Vec3d& v,
                                          const double t) const {
  return m_impl->getReactForce(p, v, t);
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

ComZmpModelSimulator& ComZmpModelSimulator::setReactForce(Functor t_functor) {
  m_impl->setReactForce(t_functor);
  return *this;
}

ComZmpModelSimulator& ComZmpModelSimulator::setReactForce(const Vec3d& t_f) {
  m_impl->setReactForce(t_f);
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

void ComZmpModelSimulator::reset() { resetTime(); }
bool ComZmpModelSimulator::update() { return update(time_step()); }
bool ComZmpModelSimulator::update(double dt) {
  updateTime(dt);
  return true;
}

}  // namespace holon
