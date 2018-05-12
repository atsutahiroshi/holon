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

class BipedFootModelSimulator : public Simulator {};

}  // namespace holon

#endif  // HOLON_HUMANOID_BIPED_FOOT_MODEL__BIPED_FOOT_MODEL_SIMULATOR_HPP_
