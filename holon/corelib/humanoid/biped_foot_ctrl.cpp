/* biped_foot_ctrl - Bipedal foot control
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of the holon.
 *
 * The holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "holon/corelib/humanoid/biped_foot_ctrl.hpp"

#include <memory>

namespace holon {

const double BipedFootCtrlParamsRawData::default_max_height = 0;
const Vec3D BipedFootCtrlParamsRawData::default_stiffness = {3000, 3000, 3000};
const Vec3D BipedFootCtrlParamsRawData::default_damping = {200, 200, 200};
BipedFootCtrlParamsRawData::BipedFootCtrlParamsRawData()
    : max_height(default_max_height) {
  stiffness = default_stiffness;
  damping = default_damping;
}

BipedFootCtrlData::BipedFootCtrlData(const Vec3D& t_initial_position)
    : DataSetBase(
          alloc_raw_data<PointMassModelRawData<Vec3D>>(1.0, t_initial_position),
          alloc_raw_data<BipedFootCtrlParamsRawData>(),
          alloc_raw_data<BipedFootCtrlOutputsRawData>(),
          alloc_raw_data<BipedFootCtrlCommandsRawData>()) {
  get<1>().position = get<0>().position;
  get<1>().velocity = get<0>().velocity;
}

BipedFootCtrl::BipedFootCtrl() {}
BipedFootCtrl::BipedFootCtrl(const Model& t_model) : Base(t_model) {}
BipedFootCtrl::BipedFootCtrl(Data t_data) : Base(t_data) {}
BipedFootCtrl::BipedFootCtrl(Data t_data, std::shared_ptr<Model> t_model_ptr)
    : Base(t_data, t_model_ptr) {}

BipedFootCtrl& BipedFootCtrl::reset() {
  model().reset();
  return *this;
}
BipedFootCtrl& BipedFootCtrl::reset(const Vec3D& t_position) {
  model().set_initial_position(t_position);
  return reset();
}

bool BipedFootCtrl::update() {
  if (!Base::update()) return false;
  return true;
}
bool BipedFootCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
  return true;
}

}  // namespace holon
