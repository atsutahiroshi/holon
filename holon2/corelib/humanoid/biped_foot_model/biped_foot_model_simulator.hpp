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

#ifndef HOLON_HUMANOID_BIPED_FOOT_MODEL__BIPED_FOOT_MODEL_SIMULATOR_HPP_
#define HOLON_HUMANOID_BIPED_FOOT_MODEL__BIPED_FOOT_MODEL_SIMULATOR_HPP_

#include <functional>
#include <memory>
#include "holon2/corelib/humanoid/biped_foot_model/biped_foot_model_data.hpp"
#include "holon2/corelib/humanoid/simulator.hpp"

namespace holon {

class BipedFootModel;

class BipedFootModelSimulator : public Simulator {
  using Self = BipedFootModelSimulator;

 public:
  using Data = BipedFootModelData;
  using Model = BipedFootModel;
  using Functor =
      std::function<Vec3d(const Vec3d&, const Vec3d&, const double)>;

 public:
  // constructors
  BipedFootModelSimulator();
  explicit BipedFootModelSimulator(const Data& t_data);
  explicit BipedFootModelSimulator(const Model& t_model);

  // special member functions
  BipedFootModelSimulator(const BipedFootModelSimulator&) = delete;
  BipedFootModelSimulator& operator=(const BipedFootModelSimulator&) = delete;
  BipedFootModelSimulator(BipedFootModelSimulator&&);
  BipedFootModelSimulator& operator=(BipedFootModelSimulator&&);
  virtual ~BipedFootModelSimulator();

  // accessors
  const Model& model() const;
  Vec3d getInitialPosition() const;

  // mutators
  Self& setInitialPosition();
  Self& setInitialPosition(const Vec3d& t_p0);

  // computations
  Vec3d getAccel(const Vec3d& p, const Vec3d& v, const double t) const;
  Vec3d getForce(const Vec3d& p, const Vec3d& v, const double t) const;

  // set functors
  Self& setForce(Functor t_functor);
  Self& setForce(const Vec3d& t_f);

  // update
  Self& reset() final;
  Self& reset(const Vec3d& t_com_position);
  bool update() final;
  bool update(double dt) final;

 private:
  class Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_FOOT_MODEL__BIPED_FOOT_MODEL_SIMULATOR_HPP_
