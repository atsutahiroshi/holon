/* phase_y - computation of phase along y-axis
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

#ifndef HOLON_HUMANOID_PHASE_Y_HPP_
#define HOLON_HUMANOID_PHASE_Y_HPP_

#include <complex>
#include "holon/corelib/math/vec3d.hpp"

namespace holon {
namespace phase_y {

using Complex = std::complex<double>;

double computeFrequency(double t_q1, double t_q2, double t_zeta);

Complex computeComplexZmp(double t_yz, double t_vy, double t_yd, double t_q1,
                          double t_q2, double t_zeta);
Complex computeComplexZmp(const Vec3D& t_zmp_position,
                          const Vec3D& t_com_velocity,
                          const Vec3D& t_ref_com_position, double t_q1,
                          double t_q2, double t_zeta);

Complex computeComplexInnerEdge(double t_yin, double t_yd, const Complex& t_pz,
                                int t_is_left);
Complex computeComplexInnerEdge(const Vec3D& t_inner_edge,
                                const Vec3D& t_ref_com_position,
                                const Complex& t_pz, int t_is_left);

double computePhase(double t_yz, double t_vy, double t_yd, double t_yin,
                    double t_q1, double t_q2, double t_zeta, int t_is_left);
double computePhase(const Vec3D& t_zmp_position, const Vec3D& t_com_velocity,
                    const Vec3D& t_ref_com_position, const Vec3D& t_inner_edge,
                    double t_q1, double t_q2, double t_zeta, int t_is_left);
double computePhase(const Complex& t_pz, const Complex& t_p0);

double computePeriod(double t_q1, double t_q2, double t_zeta);

double computeTimeSpan(const Complex& t_p0, double t_q1, double t_q2,
                       double t_zeta);
double computeElapsedTime(const Complex& t_pz, const Complex& t_p0, double t_q1,
                          double t_q2, double t_zeta);
double computeRemainingTime(const Complex& t_pz, const Complex& t_p0,
                            double t_q1, double t_q2, double t_zeta);

}  // namespace phase_y
}  // namespace holon

#endif  // HOLON_HUMANOID_PHASE_Y_HPP_
