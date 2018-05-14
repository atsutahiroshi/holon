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

#ifndef HOLON_HUMANOID_BIPED_FOOT_CONTROLLER_HPP_
#define HOLON_HUMANOID_BIPED_FOOT_CONTROLLER_HPP_

#include "holon2/corelib/humanoid/biped_foot_controller/biped_foot_controller_data.hpp"
#include "holon2/corelib/humanoid/biped_foot_model.hpp"

namespace holon {

class BipedFootController {
  using Self = BipedFootController;
  using States = BipedFootModelStates;
  using Params = BipedFootControllerParams;
  using Outputs = BipedFootControllerOutputs;

 public:
  using Data = BipedFootControllerData;
  using Model = BipedFootModelSimulator::Model;
  using Functor = BipedFootModelSimulator::Functor;

 public:
  static const Vec3d default_stiffness;
  static const Vec3d default_damping;
  static const double default_max_height;

 public:
  BipedFootController();
  BipedFootController(const Data& t_data);
  BipedFootController(const Model& t_model);

  // special member functions
  BipedFootController(const BipedFootController&) = delete;
  BipedFootController(BipedFootController&&) = delete;
  BipedFootController& operator=(const BipedFootController&) = delete;
  BipedFootController& operator=(BipedFootController&&) = delete;
  virtual ~BipedFootController() = default;

  // accessors
  const Data& data() const { return m_data; }
  const Model& model() const { return m_sim.model(); }
  double mass() const { return model().mass(); }
  const States& states() const { return model().states(); }
  const Params& params() const { return m_data.get<2>(); }
  const Outputs& outputs() const { return m_data.get<3>(); }

  // accessors to simulator
  double time() const { return m_sim.time(); }
  double time_step() const { return m_sim.time_step(); }
  Vec3d getInitialPosition() const { return m_sim.getInitialPosition(); }

  // mutators
  Self& setTimeStep(double t_time_step);

  // reset functions
  Self& reset();
  Self& reset(const Vec3d& t_position);

  // feedback functions
  Self& feedback(const Model& t_model);
  Self& feedback(const BipedFootModelData& t_model_data);

  // update functions
  bool update();
  bool update(double t_time_step);

 protected:
  States& states() {
    return const_cast<States&>(static_cast<const Self&>(*this).states());
  }
  Params& params() {
    return const_cast<Params&>(static_cast<const Self&>(*this).params());
  }
  Outputs& outputs() {
    return const_cast<Outputs&>(static_cast<const Self&>(*this).outputs());
  }
  void copyModelData(const Model& t_model);

  Functor getForceFunctor() const;

 private:
  void setDefaultParameters();
  void setupSimulator();
  void updateOutputs();

 private:
  Data m_data;
  BipedFootModelSimulator m_sim;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_FOOT_CONTROLLER_HPP_
