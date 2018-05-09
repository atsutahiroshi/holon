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
    DatasetCat<ComZmpModelData,
               Dataset<ComControllerParams, ComControllerOutputs>>;

class ComController {
  using Self = ComController;
  using Params = ComControllerParams;
  using Outputs = ComControllerOutputs;

 public:
  using Data = ComControllerData;
  using Model = ComZmpModelSimulator::Model;
  using Functor = ComZmpModelSimulator::Functor;

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
  const Params& params() const;
  const Outputs& outputs() const;

  // reset functions
  Self& reset();
  Self& reset(const Vec3d& t_com_position);

  // feedback functions
  Self& feedback(const Model& t_model);
  Self& feedback(ComZmpModelData t_model_data);
  Self& feedback(const Vec3d& t_com_position, const Vec3d& t_com_velocity);

  // update functions
  bool update();
  bool update(double t_time_step);

  // functor getter functions
  Functor getReactForceFunctor() const;
  Functor getZmpPositionFunctor() const;

 private:
  Data m_data;
  ComZmpModelSimulator m_sim;

  void copyModelData(const Model& t_model);
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_CONTROLLER_HPP_
