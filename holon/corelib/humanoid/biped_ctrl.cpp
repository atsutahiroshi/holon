/* biped_ctrl - Biped robot control
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

#include "holon/corelib/humanoid/biped_ctrl.hpp"

#include <memory>

namespace holon {

BipedCtrl::BipedCtrl() : BipedCtrl(make_data<Data>()) {}
BipedCtrl::BipedCtrl(Data t_data)
    : CtrlBase(t_data),
      m_trunk(data().extract<ComCtrlData>(Data::TrunkDataIndex()),
              model().trunk_ptr()),
      m_left_foot(data().extract<FootCtrlData>(Data::LeftFootDataIndex()),
                  model().left_foot_ptr()),
      m_right_foot(data().extract<FootCtrlData>(Data::RightFootDataIndex()),
                   model().right_foot_ptr()) {}
BipedCtrl::BipedCtrl(const Model& t_model)
    : CtrlBase(t_model),
      m_trunk(data().extract<ComCtrlData>(Data::TrunkDataIndex()),
              model().trunk_ptr()),
      m_left_foot(data().extract<FootCtrlData>(Data::LeftFootDataIndex()),
                  model().left_foot_ptr()),
      m_right_foot(data().extract<FootCtrlData>(Data::RightFootDataIndex()),
                   model().right_foot_ptr()) {}
BipedCtrl::BipedCtrl(Data t_data, std::shared_ptr<Model> t_model_ptr)
    : CtrlBase(t_data, t_model_ptr),
      m_trunk(data().extract<ComCtrlData>(Data::TrunkDataIndex()),
              model().trunk_ptr()),
      m_left_foot(data().extract<FootCtrlData>(Data::LeftFootDataIndex()),
                  model().left_foot_ptr()),
      m_right_foot(data().extract<FootCtrlData>(Data::RightFootDataIndex()),
                   model().right_foot_ptr()) {}

BipedCtrl& BipedCtrl::reset() {
  return reset(m_trunk.initial_com_position(), m_left_foot.initial_position(),
               m_right_foot.initial_position());
}
BipedCtrl& BipedCtrl::reset(const Vec3D& t_com_position, double t_foot_dist) {
  return reset(t_com_position,
               {t_com_position.x(), t_com_position.y() + 0.5 * t_foot_dist, 0},
               {t_com_position.x(), t_com_position.y() - 0.5 * t_foot_dist, 0});
}
BipedCtrl& BipedCtrl::reset(const Vec3D& t_com_position,
                            const Vec3D& t_left_foot_position,
                            const Vec3D& t_right_foot_position) {
  CtrlBase::reset();
  auto foot_dist = (t_left_foot_position - t_right_foot_position).norm();
  m_trunk.reset(t_com_position, foot_dist);
  m_left_foot.reset(t_left_foot_position);
  m_right_foot.reset(t_right_foot_position);
  return *this;
}

bool BipedCtrl::update() {
  if (!model().update()) return false;
  return true;
}
bool BipedCtrl::update(double t_time_step) {
  set_time_step(t_time_step);
  return update();
}

}  // namespace holon
