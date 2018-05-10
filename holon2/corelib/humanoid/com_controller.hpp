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

#ifndef HOLON_HUMANOID_COM_CONTROLLER_HPP_
#define HOLON_HUMANOID_COM_CONTROLLER_HPP_

#include "holon2/corelib/common/types.hpp"
#include "holon2/corelib/dataset/dataset.hpp"
#include "holon2/corelib/humanoid/com_zmp_model.hpp"
#include "holon2/corelib/math/vec.hpp"

namespace holon {

struct ComControllerParams {
  Vec3d com_position;
  Vec3d com_velocity;
  Array3d q1, q2;
  double rho, dist, kr;
};
struct ComControllerOutputs {
  Vec3d com_position;
  Vec3d com_velocity;
  Vec3d com_acceleration;
  Vec3d zmp_position;
  Vec3d reaction_force;
};
using ComControllerData =
    MultiDataset<ComZmpModelData,
                 Dataset<ComControllerParams, ComControllerOutputs>>;

class ComController {
  using Self = ComController;
  using States = ComZmpModelStates;
  using Params = ComControllerParams;
  using Outputs = ComControllerOutputs;

 public:
  using Data = ComControllerData;
  using Model = ComZmpModelSimulator::Model;
  using Functor = ComZmpModelSimulator::Functor;

 public:
  static const Array3d default_q1;
  static const Array3d default_q2;
  static const double default_rho;
  static const double default_dist;
  static const double default_kr;

 public:
  ComController();
  ComController(Data t_data);
  ComController(const Model& t_model);

  // special member functions
  ComController(const ComController&) = delete;
  ComController(ComController&&) = delete;
  ComController& operator=(const ComController&) = delete;
  ComController& operator=(ComController&&) = delete;
  virtual ~ComController() = default;

  // accessors
  const Data& data() const { return m_data; }
  const Model& model() const { return m_sim.model(); }
  double mass() const { return model().mass(); }
  double vhp() const { return model().vhp(); }
  const States& states() const { return model().states(); }
  const Params& params() const { return m_data.get<2>(); }
  const Outputs& outputs() const { return m_data.get<3>(); }

  // accessors to simulator
  double time() const { return m_sim.time(); }
  double time_step() const { return m_sim.time_step(); }
  Vec3d getInitialComPosition() const { return m_sim.getInitialComPosition(); }

  // mutators
  Self& setTimeStep(double t_time_step);

  // reset functions
  Self& reset();
  Self& reset(const Vec3d& t_com_position);

  // feedback functions
  Self& feedback(const Model& t_model);
  Self& feedback(const ComZmpModelData& t_model_data);

  // update functions
  bool update();
  bool update(double t_time_step);

  // functor getter functions
  Functor getReactForceFunctor() const;
  Functor getZmpPositionFunctor() const;

 protected:
  States& states() {
    return const_cast<States&>(static_cast<const Self&>(*this).states());
  }
  Params& params() {
    return const_cast<Params&>(static_cast<const Self&>(*this).params());
  }
  void copyModelData(const Model& t_model);

 private:
  Data m_data;
  ComZmpModelSimulator m_sim;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CONTROLLER_HPP_
