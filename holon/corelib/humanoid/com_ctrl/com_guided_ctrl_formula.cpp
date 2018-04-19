/* com_guided_ctrl_formula - Formulae for COM-guided control
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

#include "holon/corelib/humanoid/com_ctrl/com_guided_ctrl_formula.hpp"

#include <roki/rk_g.h>
#include <algorithm>
#include "holon/corelib/humanoid/com_ctrl.hpp"
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"
#include "holon/corelib/math/misc.hpp"

namespace holon {
namespace com_guided_ctrl_formula {

namespace cz = com_zmp_model_formula;

double desired_zmp_position_x(double t_x, double t_v, double t_xd, double t_vd,
                              double t_q1, double t_q2, double t_zeta) {
  if (zIsTiny(t_zeta) || t_zeta < 0) {
    ZRUNERROR("ZETA should be positive. (given: %f)", t_zeta);
    return 0;
  }
  double x = t_x - t_xd;
  double v = t_v - t_vd;
  return t_x + (t_q1 * t_q2) * x + (t_q1 + t_q2) * v / t_zeta;
}

namespace detail_y {

double nonlinear_damping(double t_y, double t_v, double t_yd, double t_q1,
                         double t_q2, double t_rho, double t_dist, double t_kr,
                         double t_zeta) {
  if (zIsTiny(t_rho) || t_rho < 0.0) return 1.0;
  if (zIsTiny(t_dist) || t_dist < 0.0) return 1.0;
  double r2 = zSqr(t_y - t_yd) + zSqr(t_v / t_zeta) / (t_q1 * t_q2);
  double rz = 0.5 * t_dist;
  return 1.0 - t_rho * exp(t_kr * (1.0 - zSqr((t_q1 * t_q2 + 1.0) / rz) * r2));
}

}  // namespace detail_y

double desired_zmp_position_y(double t_y, double t_v, double t_yd, double t_q1,
                              double t_q2, double t_rho, double t_dist,
                              double t_kr, double t_zeta) {
  if (zIsTiny(t_zeta) || t_zeta < 0) {
    ZRUNERROR("ZETA should be positive. (given: %f)", t_zeta);
    return 0;
  }
  double nd = detail_y::nonlinear_damping(t_y, t_v, t_yd, t_q1, t_q2, t_rho,
                                          t_dist, t_kr, t_zeta);
  return t_y + (t_q1 * t_q2) * (t_y - t_yd) + (t_q1 + t_q2) * nd * t_v / t_zeta;
}

Vec3D desired_zmp_position(const Vec3D& t_com_position,
                           const Vec3D& t_com_velocity,
                           const Vec3D& t_ref_com_position,
                           const Vec3D& t_ref_com_velocity, const Array3d& t_q1,
                           const Array3d& t_q2, double t_rho, double t_dist,
                           double t_kr, double t_vhp, double t_zeta) {
  using namespace index_symbols;
  auto xz = desired_zmp_position_x(
      t_com_position[_X], t_com_velocity[_X], t_ref_com_position[_X],
      t_ref_com_velocity[_X], t_q1[_X], t_q2[_X], t_zeta);
  auto yz = desired_zmp_position_y(t_com_position[_Y], t_com_velocity[_Y],
                                   t_ref_com_position[_Y], t_q1[_Y], t_q2[_Y],
                                   t_rho, t_dist, t_kr, t_zeta);
  auto zz = t_vhp;
  return Vec3D{xz, yz, zz};
}

Vec3D desired_zmp_position(const Vec3D& t_com_position,
                           const Vec3D& t_com_velocity,
                           const ComCtrlParamsRawData& t_params,
                           double t_zeta) {
  return desired_zmp_position(t_com_position, t_com_velocity,
                              t_params.com_position, t_params.com_velocity,
                              t_params.q1, t_params.q2, t_params.rho,
                              t_params.dist, t_params.kr, t_params.vhp, t_zeta);
}

namespace detail_z {

double squared_xi(double t_zd) {
  if (zIsTiny(t_zd) || t_zd < 0) {
    ZRUNERROR("Desired COM height should be positive. (given: %f)", t_zd);
    return 0;
  }
  return RK_G / t_zd;
}

}  // namespace detail_z

double desired_reaction_force_z(double t_z, double t_v, double t_zd,
                                double t_q1, double t_q2, double t_mass) {
  double xi2 = detail_z::squared_xi(t_zd);
  double xi = sqrt(xi2);
  double fz =
      -xi2 * t_q1 * t_q2 * (t_z - t_zd) - xi * (t_q1 + t_q2) * t_v + RK_G;
  fz *= t_mass;
  return std::max<double>(fz, 0);
}
double desired_reaction_force_z(const Vec3D& t_com_position,
                                const Vec3D& t_com_velocity,
                                const ComCtrlParamsRawData& t_params) {
  using namespace index_symbols;
  return desired_reaction_force_z(t_com_position[_Z], t_com_velocity[_Z],
                                  t_params.com_position[_Z], t_params.q1[_Z],
                                  t_params.q2[_Z], t_params.mass);
}

double frequency(double t_q1, double t_q2, double t_zeta) {
  if (t_zeta < 0 || (t_q1 * t_q2) < 0) {
    ZRUNERROR("ZETA, Q1 and Q2 must be positive. (given: %f, %f, %f)", t_zeta,
              t_q1, t_q2);
    return 0;
  }
  return t_zeta * std::sqrt(t_q1 * t_q2);
}

Complex complex_zmp_y(double t_yz, double t_vy, double t_yd, double t_q1,
                      double t_q2, double t_zeta) {
  double omega = frequency(t_q1, t_q2, t_zeta);
  return Complex(t_yz - t_yd, -(t_q1 * t_q2 + 1.0) * t_vy / omega);
}

Complex complex_zmp_y(const Vec3D& t_zmp_position, const Vec3D& t_com_velocity,
                      const ComCtrlParamsRawData& t_params, double t_zeta) {
  using namespace index_symbols;
  return complex_zmp_y(t_zmp_position[_Y], t_com_velocity[_Y],
                       t_params.com_position[_Y], t_params.q1[_Y],
                       t_params.q2[_Y], t_zeta);
}

Complex complex_inner_edge(double t_yin, double t_yd, const Complex& t_pz,
                           BipedFootType t_type) {
  double y = t_yin - t_yd;
  return Complex(
      y, -std::sqrt(std::norm(t_pz) - y * y) * sgn(static_cast<int>(t_type)));
}

Complex complex_inner_edge(const Vec3D& t_inner_edge,
                           const ComCtrlParamsRawData& t_params,
                           const Complex& t_pz, BipedFootType t_type) {
  using namespace index_symbols;
  return complex_inner_edge(t_inner_edge[_Y], t_params.com_position[_Y], t_pz,
                            t_type);
}

double phase_y(double t_yz, double t_vy, double t_yd, double t_yin, double t_q1,
               double t_q2, double t_zeta, BipedFootType t_type) {
  auto pz = complex_zmp_y(t_yz, t_vy, t_yd, t_q1, t_q2, t_zeta);
  return phase_y(pz, complex_inner_edge(t_yin, t_yd, pz, t_type));
}

double phase_y(const Complex& t_pz, const Complex& t_p0) {
  auto t_p1 = std::conj(t_p0);
  auto denom = std::arg(t_p1 / t_p0);
  if (denom < 0) denom += 2.0 * M_PI;
  auto numer = std::arg(t_pz / t_p0);
  if (zIsTiny(denom - std::fabs(numer))) return 1.0;
  return limit(numer / denom, 0.0, 1.0);
}

namespace detail_phase {

bool is_omega_valid(const double t_omega) {
  if (zIsTiny(t_omega)) {
    ZRUNERROR("Frequency of oscillation must be positive. (given: %f)",
              t_omega);
    return false;
  }
  return true;
}

bool is_omega_valid(const double t_q1, const double t_q2, const double t_zeta) {
  return is_omega_valid(frequency(t_q1, t_q2, t_zeta));
}

}  // namespace detail_phase

double period(double t_q1, double t_q2, double t_zeta) {
  auto omega = frequency(t_q1, t_q2, t_zeta);
  if (!detail_phase::is_omega_valid(omega)) return 0;
  return zPIx2 / omega;
}

double time_span(const Complex& t_p0, double t_q1, double t_q2, double t_zeta) {
  auto omega = frequency(t_q1, t_q2, t_zeta);
  if (!detail_phase::is_omega_valid(omega)) return 0;
  return std::arg(std::conj(t_p0) / t_p0) / omega;
}

double elapsed_time(const Complex& t_pz, const Complex& t_p0, double t_q1,
                    double t_q2, double t_zeta) {
  auto omega = frequency(t_q1, t_q2, t_zeta);
  if (!detail_phase::is_omega_valid(omega)) return 0;
  return std::arg(t_pz / t_p0) / omega;
}

double remaining_time(const Complex& t_pz, const Complex& t_p0, double t_q1,
                      double t_q2, double t_zeta) {
  auto omega = frequency(t_q1, t_q2, t_zeta);
  if (!detail_phase::is_omega_valid(omega)) return 0;
  return std::arg(std::conj(t_p0) / t_pz) / omega;
}

}  // namespace com_guided_ctrl_formula
}  // namespace holon
