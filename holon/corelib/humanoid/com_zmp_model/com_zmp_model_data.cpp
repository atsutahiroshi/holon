/* com_zmp_model_data - Data for COM-ZMP model
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

#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_data.hpp"

#include <roki/rk_g.h>
#include <memory>
#include <utility>

namespace holon {

const double ComZmpModelData::default_mass = 1.0;
const Vec3D ComZmpModelData::default_com_position = {0.0, 0.0, 1.0};

ComZmpModelData::ComZmpModelData(const RawData& t_raw_data)
    : Base(t_raw_data) {}
ComZmpModelData::ComZmpModelData(std::shared_ptr<RawData> t_raw_data_p)
    : Base(t_raw_data_p) {}
ComZmpModelData::ComZmpModelData(const Vec3D& t_com_position, double t_mass)
    : ComZmpModelData(std::move(ComZmpModelRawData({t_mass,
                                                    kVec3DZ,
                                                    t_com_position,
                                                    kVec3DZero,
                                                    kVec3DZero,
                                                    kVec3DZero,
                                                    {0, 0, t_mass * RK_G},
                                                    kVec3DZero,
                                                    {0, 0, t_mass * RK_G}}))) {}

}  // namespace holon
