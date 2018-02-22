/* com_ctrl_z - COM controller along z axis
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

#include "holon/corelib/humanoid/com_ctrl_z.hpp"

#include <cure/cure_misc.h>
#include <roki/rk_g.h>

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

const double G = RK_G;
const double SQRT_G = sqrt(G);

TEST_CASE("ComCtrlZ(): constructor", "[corelib][humanoid][ComCtrlZ]") {
  SECTION("default constructor") {
    ComCtrlZ ctrl;
    CHECK(ctrl.q1() == 1.0);
    CHECK(ctrl.q2() == 1.0);
  }

  SECTION("with arguments") {
    Fuzzer fuzz(0, 1);
    double q1 = fuzz();
    double q2 = fuzz();
    ComCtrlZ ctrl(q1, q2);
    CHECK(ctrl.q1() == q1);
    CHECK(ctrl.q2() == q2);
  }
}

TEST_CASE("accessors/mutators", "[corelib][humanoid][ComCtrlZ]") {
  Fuzzer fuzz(0, 1);
  ComCtrlZ ctrl;

  SECTION("q1") {
    double q1 = fuzz();
    REQUIRE(ctrl.q1() != q1);
    ctrl.set_q1(q1);
    CHECK(ctrl.q1() == q1);
  }

  SECTION("q2") {
    double q2 = fuzz();
    REQUIRE(ctrl.q2() != q2);
    ctrl.set_q2(q2);
    CHECK(ctrl.q2() == q2);
  }
}

TEST_CASE("ComCtrlZ::computeDesReactForce(double,double,double,double)",
          "[corelib][humanoid][ComCtrlZ]") {
  ComCtrlZ ctrl;

  SECTION("case: zd = 1, q1 = 1, q2 = 1, m = 1") {
    double zd = 1;
    double m = 1;
    REQUIRE(ctrl.q1() == 1);
    REQUIRE(ctrl.q2() == 1);

    struct testcase_t {
      double z, vz;
      double expected_fz;
    } testcases[] = {{1, 0, G},
                     {0.5, 0.5, 1.5 * G - SQRT_G},
                     {1.5, -1, 0.5 * G + 2. * SQRT_G},
                     {0.8, -0.5, 1.2 * G + SQRT_G}};

    for (const auto& c : testcases) {
      INFO("z = " << c.z << ", vz = " << c.vz);
      CHECK(ctrl.computeDesReactForce(zd, c.z, c.vz, m) ==
            Approx(c.expected_fz));
    }
  }

#define XI2(zd) (RK_G / zd)
#define XI(zd) sqrt(XI2(zd))
  SECTION("case: zd = 0.42, q1 = 0.5, q2 = 1, m = 1") {
    double zd = 0.42;
    double m = 1;
    ctrl.set_q1(0.5);
    REQUIRE(ctrl.q1() == 0.5);
    REQUIRE(ctrl.q2() == 1);

    struct testcase_t {
      double z, vz;
      double expected_fz;
    } testcases[] = {{0.46, 0, 20. * G / 21},
                     {0.42, -0.5, 0.75 * XI(zd) + G},
                     {0.4, 0.1, 0.01 * XI2(zd) - 0.15 * XI(zd) + G},
                     {0.4, -0.1, 0.01 * XI2(zd) + 0.15 * XI(zd) + G}};

    for (const auto& c : testcases) {
      INFO("z = " << c.z << ", vz = " << c.vz);
      CHECK(ctrl.computeDesReactForce(zd, c.z, c.vz, m) ==
            Approx(c.expected_fz));
    }
  }

  SECTION("case: zd = 0.42, q1 = 1, q2 = 0.5, m = 1.5") {
    double zd = 0.42;
    double m = 1.5;
    ctrl.set_q2(0.5);
    REQUIRE(ctrl.q1() == 1);
    REQUIRE(ctrl.q2() == 0.5);

    struct testcase_t {
      double z, vz;
      double expected_fz;
    } testcases[] = {{0.46, 0, 30. * G / 21},
                     {0.42, -0.5, 1.125 * XI(zd) + 1.5 * G},
                     {0.4, 0.1, 0.015 * XI2(zd) - 0.225 * XI(zd) + 1.5 * G},
                     {0.4, -0.1, 0.015 * XI2(zd) + 0.225 * XI(zd) + 1.5 * G}};

    for (const auto& c : testcases) {
      INFO("z = " << c.z << ", vz = " << c.vz);
      CHECK(ctrl.computeDesReactForce(zd, c.z, c.vz, m) ==
            Approx(c.expected_fz));
    }
  }

  SECTION("case: zd = 0.42, q1 = 1, q2 = 1, m = 1") {
    double zd = 0.42;
    double m = 1;
    REQUIRE(ctrl.q1() == 1);
    REQUIRE(ctrl.q2() == 1);

    struct testcase_t {
      double z, vz;
      double expected_fz;
    } testcases[] = {{0.46, 1, 0}};

    for (const auto& c : testcases) {
      INFO("If negative reaction force is needed it should be 0")
      INFO("z = " << c.z << ", vz = " << c.vz);
      CHECK(ctrl.computeDesReactForce(zd, c.z, c.vz, m) ==
            Approx(c.expected_fz));
    }
  }
}

TEST_CASE(
    "ComCtrlZ::computeDesReactForce"
    "(const Vec3D&,const Vec3D&,const Vec3D&,double)",
    "[corelib][humanoid][ComCtrlZ]") {
  ComCtrlZ ctrl;

  SECTION("case: zd = 0.42, q1 = 1, q2 = 0.5, m = 1.5") {
    double zd = 0.42;
    double m = 1.5;
    ctrl.set_q2(0.5);
    REQUIRE(ctrl.q1() == 1);
    REQUIRE(ctrl.q2() == 0.5);

    struct testcase_t {
      double z, vz;
      double expected_fz;
    } testcases[] = {{0.46, 0, 30. * G / 21},
                     {0.42, -0.5, 1.125 * XI(zd) + 1.5 * G},
                     {0.4, 0.1, 0.015 * XI2(zd) - 0.225 * XI(zd) + 1.5 * G},
                     {0.4, -0.1, 0.015 * XI2(zd) + 0.225 * XI(zd) + 1.5 * G}};

    for (const auto& c : testcases) {
      Vec3D ref_com_pos = {0, 0, zd};
      Vec3D com_pos = {0, 0, c.z};
      Vec3D com_vel = {0, 0, c.vz};
      INFO("z = " << c.z << ", vz = " << c.vz);
      CHECK(ctrl.computeDesReactForce(ref_com_pos, com_pos, com_vel, m) ==
            Approx(c.expected_fz));
    }
  }
}

}  // namespace
}  // namespace holon
