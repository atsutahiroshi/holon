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

#include <memory>
#include "holon2/corelib/dataset/dataset.hpp"
#include "holon2/corelib/humanoid/simulator.hpp"

namespace holon {

struct ComZmpModelParams;
struct ComZmpModelStates;
using ComZmpModelData = Dataset<ComZmpModelParams, ComZmpModelStates>;
class ComZmpModel;

class ComZmpModelSimulator : public Simulator {
  using Data = ComZmpModelData;
  using Model = ComZmpModel;

 public:
  // constructors
  ComZmpModelSimulator();
  ComZmpModelSimulator(Data t_data);
  ComZmpModelSimulator(const Model& t_model);

  // special member functions
  ComZmpModelSimulator(const ComZmpModelSimulator&) = delete;
  ComZmpModelSimulator& operator=(const ComZmpModelSimulator&) = delete;
  ComZmpModelSimulator(ComZmpModelSimulator&&);
  ComZmpModelSimulator& operator=(ComZmpModelSimulator&&);
  virtual ~ComZmpModelSimulator();

  void reset() override { resetTime(); }
  bool update() override { return update(time_step()); }
  bool update(double dt) override {
    updateTime(dt);
    return true;
  }

 private:
  class Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_COM_ZMP_MODEL_SIMULATOR_HPP_
