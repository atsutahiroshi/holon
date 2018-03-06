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

#include "holon/corelib/humanoid/com_ctrl/phase_y.hpp"

#include <zm/zm_misc.h>
#include <cmath>
#include <utility>

namespace holon {
namespace phase_y {

double computeFrequency(double t_q1, double t_q2, double t_zeta) {
  if (t_zeta < 0 || (t_q1 * t_q2) < 0) {
    ZRUNERROR("ZETA, Q1 and Q2 must be positive. (given: %f, %f, %f)", t_zeta,
              t_q1, t_q2);
    return 0;
  }
  return t_zeta * std::sqrt(t_q1 * t_q2);
}

Complex computeComplexZmp(double t_yz, double t_vy, double t_yd, double t_q1,
                          double t_q2, double t_zeta) {
  double omega = computeFrequency(t_q1, t_q2, t_zeta);
  return Complex(t_yz - t_yd, -(t_q1 * t_q2 + 1.0) * t_vy / omega);
}

Complex computeComplexZmp(const Vec3D& t_zmp_position,
                          const Vec3D& t_com_velocity,
                          const Vec3D& t_ref_com_position, double t_q1,
                          double t_q2, double t_zeta) {
  return computeComplexZmp(t_zmp_position.y(), t_com_velocity.y(),
                           t_ref_com_position.y(), t_q1, t_q2, t_zeta);
}

namespace internal {

template <typename T>
int sgn(const T& x) {
  return (T(0) < x) - (x < T(0));
}

}  // namespace internal

Complex computeComplexInnerEdge(double t_yin, double t_yd, const Complex& t_pz,
                                int t_is_left) {
  double y = t_yin - t_yd;
  return Complex(
      y, -std::sqrt(std::norm(t_pz) - y * y) * internal::sgn(t_is_left));
}

Complex computeComplexInnerEdge(const Vec3D& t_inner_edge,
                                const Vec3D& t_ref_com_position,
                                const Complex& t_pz, int t_is_left) {
  return computeComplexInnerEdge(t_inner_edge.y(), t_ref_com_position.y(), t_pz,
                                 t_is_left);
}

double computePhase(double t_yz, double t_vy, double t_yd, double t_yin,
                    double t_q1, double t_q2, double t_zeta, int t_is_left) {
  auto pz = computeComplexZmp(t_yz, t_vy, t_yd, t_q1, t_q2, t_zeta);
  return computePhase(pz, computeComplexInnerEdge(t_yin, t_yd, pz, t_is_left));
}
double computePhase(const Vec3D& t_zmp_position, const Vec3D& t_com_velocity,
                    const Vec3D& t_ref_com_position, const Vec3D& t_inner_edge,
                    double t_q1, double t_q2, double t_zeta, int t_is_left) {
  auto pz = computeComplexZmp(t_zmp_position, t_com_velocity,
                              t_ref_com_position, t_q1, t_q2, t_zeta);
  return computePhase(pz, computeComplexInnerEdge(
                              t_inner_edge, t_ref_com_position, pz, t_is_left));
}

namespace internal {

template <typename T>
T limit(const T& x, const T& lower, const T& upper) {
  return x <= lower ? lower : ((x >= upper) ? upper : x);
}

}  // namespace internal

double computePhase(const Complex& t_pz, const Complex& t_p0) {
  auto t_p1 = std::conj(t_p0);
  auto denom = std::arg(t_p1 / t_p0);
  if (denom < 0) denom += 2.0 * M_PI;
  auto numer = std::arg(t_pz / t_p0);
  if (zIsTiny(denom - std::fabs(numer))) return 1.0;
  return internal::limit(numer / denom, 0.0, 1.0);
}

namespace internal {

bool isOmegaValid(const double t_omega) {
  if (zIsTiny(t_omega)) {
    ZRUNERROR("Frequency of oscillation must be positive. (given: %f)",
              t_omega);
    return false;
  }
  return true;
}

bool isOmegaValid(const double t_q1, const double t_q2, const double t_zeta) {
  return isOmegaValid(computeFrequency(t_q1, t_q2, t_zeta));
}

}  // namespace internal

double computePeriod(double t_q1, double t_q2, double t_zeta) {
  auto omega = computeFrequency(t_q1, t_q2, t_zeta);
  if (!internal::isOmegaValid(omega)) return 0;
  return zPIx2 / omega;
}

double computeTimeSpan(const Complex& t_p0, double t_q1, double t_q2,
                       double t_zeta) {
  auto omega = computeFrequency(t_q1, t_q2, t_zeta);
  if (!internal::isOmegaValid(omega)) return 0;
  return std::arg(std::conj(t_p0) / t_p0) / omega;
}

double computeElapsedTime(const Complex& t_pz, const Complex& t_p0, double t_q1,
                          double t_q2, double t_zeta) {
  auto omega = computeFrequency(t_q1, t_q2, t_zeta);
  if (!internal::isOmegaValid(omega)) return 0;
  return std::arg(t_pz / t_p0) / omega;
}

double computeRemainingTime(const Complex& t_pz, const Complex& t_p0,
                            double t_q1, double t_q2, double t_zeta) {
  auto omega = computeFrequency(t_q1, t_q2, t_zeta);
  if (!internal::isOmegaValid(omega)) return 0;
  return std::arg(std::conj(t_p0) / t_pz) / omega;
}

}  // namespace phase_y
}  // namespace holon
