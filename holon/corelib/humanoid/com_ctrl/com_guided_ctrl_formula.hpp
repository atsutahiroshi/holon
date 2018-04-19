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

#ifndef HOLON_HUMANOID_COM_GUIDED_CTRL_FORMULA_HPP_
#define HOLON_HUMANOID_COM_GUIDED_CTRL_FORMULA_HPP_

#include <array>
#include <complex>
#include "holon/corelib/common/types.hpp"
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

struct ComCtrlParamsRawData;
enum class BipedFootType;

namespace com_guided_ctrl_formula {

// computations of desired ZMP position
double desired_zmp_position_x(double t_x, double t_v, double t_xd, double t_vd,
                              double t_q1, double t_q2, double t_zeta);
double desired_zmp_position_y(double t_y, double t_v, double t_yd, double t_q1,
                              double t_q2, double t_rho, double t_dist,
                              double t_kr, double t_zeta);
Vec3D desired_zmp_position(const Vec3D& t_com_position,
                           const Vec3D& t_com_velocity,
                           const Vec3D& t_ref_com_position,
                           const Vec3D& t_ref_com_velocity, const Array3d& t_q1,
                           const Array3d& t_q2, double t_rho, double t_dist,
                           double t_kr, double t_vhp, double t_zeta);
Vec3D desired_zmp_position(const Vec3D& t_com_position,
                           const Vec3D& t_com_velocity,
                           const ComCtrlParamsRawData& t_params, double t_zeta);

// compuations of desired reaction force along z-axis
double desired_reaction_force_z(double t_z, double t_v, double t_zd,
                                double t_q1, double t_q2, double t_mass);
double desired_reaction_force_z(const Vec3D& t_com_position,
                                const Vec3D& t_com_velocity,
                                const ComCtrlParamsRawData& t_params);

// computations related to sideward oscillation
double frequency(double t_q1, double t_q2, double t_zeta);
double period(double t_q1, double t_q2, double t_zeta);

// computations related to oscillation phase
Complex complex_zmp_y(double t_yz, double t_vy, double t_yd, double t_q1,
                      double t_q2, double t_zeta);
Complex complex_zmp_y(const Vec3D& t_zmp_position, const Vec3D& t_com_velocity,
                      const ComCtrlParamsRawData& t_params, double t_zeta);
Complex complex_inner_edge(double t_yin, double t_yd, const Complex& t_pz,
                           BipedFootType t_type);
Complex complex_inner_edge(const Vec3D& t_inner_edge,
                           const ComCtrlParamsRawData& t_params,
                           const Complex& t_pz, BipedFootType t_type);
double phase_y(double t_yz, double t_vy, double t_yd, double t_yin, double t_q1,
               double t_q2, double t_zeta, BipedFootType t_type);
double phase_y(const Complex& t_pz, const Complex& t_p0);

double time_span(const Complex& t_p0, double t_q1, double t_q2, double t_zeta);
double elapsed_time(const Complex& t_pz, const Complex& t_p0, double t_q1,
                    double t_q2, double t_zeta);
double remaining_time(const Complex& t_pz, const Complex& t_p0, double t_q1,
                      double t_q2, double t_zeta);

}  // namespace com_guided_ctrl_formula
}  // namespace holon

#endif  // HOLON_HUMANOID_COM_GUIDED_CTRL_FORMULA_HPP_
