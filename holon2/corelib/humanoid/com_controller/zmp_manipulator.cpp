/* zmp_manipulator - ZMP manipulator class
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

#include "holon2/corelib/humanoid/com_controller/zmp_manipulator.hpp"

#include "holon2/corelib/math/misc.hpp"

namespace holon {

ZmpManipulator::ZmpManipulator(const Data& t_data) : m_data(t_data) {}

ZmpManipulator& ZmpManipulator::setData(const Data& t_data) {
  m_data = t_data;
  return *this;
}

double ZmpManipulator::calculateX(const double t_x, const double t_v,
                                  const double t_zeta) {
  if (isTiny(t_zeta) || t_zeta < 0) {
    // ZRUNERROR("ZETA should be positive. (given: %f)", t_zeta);
    return 0;
  }
  double x = t_x - params().com_position[0];
  double v = t_v - params().com_velocity[0];
  double k1 = params().q1[0] * params().q2[0];
  double k2 = (params().q1[0] + params().q2[0]) / t_zeta;
  return t_x + k1 * x + k2 * v;
}

namespace {

double calculateNonlinearDamping(double t_y, double t_v, double t_yd,
                                 double t_q1, double t_q2, double t_rho,
                                 double t_dist, double t_kr, double t_zeta) {
  if (isTiny(t_rho) || t_rho < 0.0) return 1.0;
  if (isTiny(t_dist) || t_dist < 0.0) return 1.0;
  double r2 = sqr(t_y - t_yd) + sqr(t_v / t_zeta) / (t_q1 * t_q2);
  double rz = 0.5 * t_dist;
  return 1.0 - t_rho * exp(t_kr * (1.0 - sqr((t_q1 * t_q2 + 1.0) / rz) * r2));
}

}  // namespace

double ZmpManipulator::calculateY(const double t_y, const double t_v,
                                  const double t_zeta) {
  if (isTiny(t_zeta) || t_zeta < 0) {
    // ZRUNERROR("ZETA should be positive. (given: %f)", t_zeta);
    return 0;
  }
  double y = t_y - params().com_position[1];
  double k1 = params().q1[1] * params().q2[1];
  double k2 = (params().q1[1] + params().q2[1]) / t_zeta;
  double nd = calculateNonlinearDamping(
      t_y, t_v, params().com_position[1], params().q1[1], params().q2[1],
      params().rho, params().dist, params().kr, t_zeta);
  return t_y + k1 * y + k2 * nd * t_v;
}

Vec3d ZmpManipulator::calculate(const Vec3d& t_com_position,
                                const Vec3d& t_com_velocity,
                                const double t_zeta) {
  auto xz = calculateX(t_com_position.x(), t_com_velocity.x(), t_zeta);
  auto yz = calculateY(t_com_position.y(), t_com_velocity.y(), t_zeta);
  auto zz = vhp();
  return Vec3d(xz, yz, zz);
}

}  // namespace holon
