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

#include "holon2/corelib/humanoid/com_controller.hpp"

namespace holon {

ComController::ComController() : ComController(Data()) {}
ComController::ComController(Data t_data)
    : m_data(t_data), m_sim(m_data.subdata<0, 1>()) {}
ComController::ComController(const Model& t_model) : ComController() {
  copyModelData(t_model);
}

void ComController::copyModelData(const Model& t_model) {
  m_data.copy(t_model.data(), IndexSeq<0, 1>(), IndexSeq<0, 1>());
}

}  // namespace holon
