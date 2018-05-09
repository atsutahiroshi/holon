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

#ifndef HOLON_HUMANOID_COM_ZMP_MODEL_SIMULATOR_HPP_
#define HOLON_HUMANOID_COM_ZMP_MODEL_SIMULATOR_HPP_

#include <functional>
#include <memory>
#include "holon2/corelib/dataset/dataset.hpp"
#include "holon2/corelib/humanoid/simulator.hpp"
#include "holon2/corelib/math/vec.hpp"

namespace holon {

struct ComZmpModelParams;
struct ComZmpModelStates;
using ComZmpModelData = Dataset<ComZmpModelParams, ComZmpModelStates>;
class ComZmpModel;

class ComZmpModelSimulator : public Simulator {
  using Self = ComZmpModelSimulator;
  using Data = ComZmpModelData;
  using Model = ComZmpModel;
  using Functor =
      std::function<Vec3d(const Vec3d&, const Vec3d&, const double)>;

 public:
  enum class InputType {
    kNotDetermined,
    kReactionForce,
    kZmpPosition,
  };

 public:
  // constructors
  ComZmpModelSimulator();
  explicit ComZmpModelSimulator(Data t_data);
  explicit ComZmpModelSimulator(const Model& t_model);

  // special member functions
  ComZmpModelSimulator(const ComZmpModelSimulator&) = delete;
  ComZmpModelSimulator& operator=(const ComZmpModelSimulator&) = delete;
  ComZmpModelSimulator(ComZmpModelSimulator&&);
  ComZmpModelSimulator& operator=(ComZmpModelSimulator&&);
  virtual ~ComZmpModelSimulator();

  // accessors
  const Model& model() const;
  InputType getInputType() const;
  Vec3d getInitialComPosition() const;

  // boolean
  bool isZmpPosAsInput() const;
  bool isReactForceAsInput() const;

  // mutators
  ComZmpModelSimulator& setInputType(InputType t_type);
  ComZmpModelSimulator& setZmpPosAsInput();
  ComZmpModelSimulator& setReactForceAsInput();
  ComZmpModelSimulator& setInitialComPosition(const Vec3d& t_p0);

  // computations
  Vec3d getComAccel(const Vec3d& p, const Vec3d& v, const double t) const;
  Vec3d getZmpPosition(const Vec3d& p, const Vec3d& v, const double t) const;
  Vec3d getReactForce(const Vec3d& p, const Vec3d& v, const double t) const;
  Vec3d getExtForce(const Vec3d& p, const Vec3d& v, const double t) const;

  // set functors
  ComZmpModelSimulator& setZmpPosition(Functor t_functor);
  ComZmpModelSimulator& setZmpPosition(const Vec3d& t_pz);
  ComZmpModelSimulator& setReactForce(Functor t_functor);
  ComZmpModelSimulator& setReactForce(const Vec3d& t_f);
  ComZmpModelSimulator& setExtForce(Functor t_functor);
  ComZmpModelSimulator& setExtForce(const Vec3d& t_ef);
  ComZmpModelSimulator& clearExtForce();

  // update
  void reset() final;
  bool update() final;
  bool update(double dt) final;

 private:
  class Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_SIMULATOR_HPP_
