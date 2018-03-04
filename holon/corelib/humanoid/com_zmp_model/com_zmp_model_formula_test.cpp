/* com_zmp_model_formula - Formulae related to COM-ZMP model
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

#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

#include <roki/rk_g.h>

#include "catch.hpp"
#include "holon/test/util/catch/custom_matchers.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

using com_zmp_model_formula::computeSqrZeta;
using com_zmp_model_formula::computeZeta;
using com_zmp_model_formula::computeReactForce;
using com_zmp_model_formula::computeComAcc;
using com_zmp_model_formula::isMassValid;
using com_zmp_model_formula::isComZmpDiffValid;
using com_zmp_model_formula::isReactionForceValid;
using com_zmp_model_formula::isComAccelerationValid;

const double G = RK_G;

TEST_CASE("computeSqrZeta(double,double,double)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  SECTION("various COM height, ZMP height and COM acc fixed to 0") {
    double zz = 0;
    double az = 0;
    struct testcaset_t {
      double z;
      double expected_sqr_zeta;
    } testcases[] = {{1, G}, {G, 1.0}, {2, G / 2}, {4, G / 4}};
    for (const auto& c : testcases) {
      INFO("z=" << c.z << ", zz=" << zz << ", az=" << az);
      CHECK(computeSqrZeta(c.z, zz, az) == c.expected_sqr_zeta);
      CHECK(computeZeta(c.z, zz, az) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various ZMP height, COM height = 2 and COM acc = 0") {
    double z = 2;
    double az = 0;
    struct testcaset_t {
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, G}, {1.5, 2.0 * G}, {0.5, 2.0 * G / 3.0}, {-0.5, 2.0 * G / 5.0}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << c.zz << ", az=" << az);
      CHECK(computeSqrZeta(z, c.zz, az) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, c.zz, az) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various COM acceleration, COM height = 1.5 and ZMP height = 0.5") {
    double z = 1.5;
    double zz = 0.5;
    struct testcaset_t {
      double az;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, 1.0 + G}, {1.5, 1.5 + G}, {-1, -1.0 + G}, {G, 2.0 * G}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", az=" << c.az);
      CHECK(computeSqrZeta(z, zz, c.az) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, c.az) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("return 0 when (z - zz) <= 0") {
    double az = 0;
    struct testcase_t {
      double z;
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1, 0}, {1, 2, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << c.z << ", zz=" << c.zz << ", az=" << az);
      CHECK(computeSqrZeta(c.z, c.zz, az) == c.expected_sqr_zeta);
      CHECK(computeZeta(c.z, c.zz, az) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when (az + g) <= 0") {
    double z = 1;
    double zz = 0;
    struct testcase_t {
      double az;
      double expected_sqr_zeta;
    } testcases[] = {{-2.0 * G, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", az=" << c.az);
      CHECK(computeSqrZeta(z, zz, c.az) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, c.az) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }
}

TEST_CASE("computeSqrZeta(double,double,double,double)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  SECTION("various COM height, fixed ZMP = 0, Fz = G, m = 1") {
    double zz = 0;
    double fz = G;
    double m = 1;
    struct testcaset_t {
      double z;
      double expected_sqr_zeta;
    } testcases[] = {{1, G}, {G, 1.0}, {2, G / 2}, {4, G / 4}};
    for (const auto& c : testcases) {
      INFO("z=" << c.z << ", zz=" << zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(c.z, zz, fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(c.z, zz, fz, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various ZMP height, fixed COM = 2, Fz = G, m = 1") {
    double z = 2;
    double fz = G;
    double m = 1;
    struct testcaset_t {
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, G}, {1.5, 2.0 * G}, {0.5, 2.0 * G / 3.0}, {-0.5, 2.0 * G / 5.0}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << c.zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(z, c.zz, fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, c.zz, fz, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various Fz, fixed COM = 2, ZMP = 1, m = 1") {
    double z = 2;
    double zz = 1;
    double m = 1;
    struct testcaset_t {
      double fz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1}, {1.5, 1.5}, {0.5, 0.5}, {G, G}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", fz=" << c.fz << ", m=" << m);
      CHECK(computeSqrZeta(z, zz, c.fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, c.fz, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various mass, fixed COM = 2, ZMP = 1, Fz = 2") {
    double z = 2;
    double zz = 1;
    double fz = 2;
    struct testcaset_t {
      double m;
      double expected_sqr_zeta;
    } testcases[] = {{1, 2}, {2, 1}, {0.5, 4}, {1.5, 4.0 / 3}};
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", fz=" << fz << ", m=" << c.m);
      CHECK(computeSqrZeta(z, zz, fz, c.m) == Approx(c.expected_sqr_zeta));
      CHECK(computeZeta(z, zz, fz, c.m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("return 0 when (z - zz) <= 0") {
    double fz = 1;
    double m = 1;
    struct testcase_t {
      double z;
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1, 0}, {1, 2, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << c.z << ", zz=" << c.zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(c.z, c.zz, fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(c.z, c.zz, fz, m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when fz < 0") {
    double z = 2;
    double zz = 1;
    double m = 1;
    struct testcase_t {
      double fz;
      double expected_sqr_zeta;
    } testcases[] = {{-1, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", fz=" << c.fz << ", m=" << m);
      CHECK(computeSqrZeta(z, zz, c.fz, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, c.fz, m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when mass < 0") {
    double z = 2;
    double zz = 1;
    double fz = 1;
    struct testcase_t {
      double m;
      double expected_sqr_zeta;
    } testcases[] = {{-1, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      INFO("z=" << z << ", zz=" << zz << ", fz=" << fz << ", m=" << c.m);
      CHECK(computeSqrZeta(z, zz, fz, c.m) == c.expected_sqr_zeta);
      CHECK(computeZeta(z, zz, fz, c.m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }
}

TEST_CASE("computeSqrZeta(Vec3D&,Vec3D&,Vec3D&)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  SECTION("various COM height, ZMP height and COM acc fixed to 0") {
    double zz = 0;
    double az = 0;
    struct testcaset_t {
      double z;
      double expected_sqr_zeta;
    } testcases[] = {{1, G}, {G, 1.0}, {2, G / 2}, {4, G / 4}};
    for (const auto& c : testcases) {
      Vec3D p{1, 2, c.z};
      Vec3D pz{0, 1, zz};
      Vec3D ddp{1, 0, az};
      INFO("z=" << c.z << ", zz=" << zz << ", az=" << az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various ZMP height, COM height = 2 and COM acc = 0") {
    double z = 2;
    double az = 0;
    struct testcaset_t {
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, G}, {1.5, 2.0 * G}, {0.5, 2.0 * G / 3.0}, {-0.5, 2.0 * G / 5.0}};
    for (const auto& c : testcases) {
      Vec3D p{1, 0, z};
      Vec3D pz{1, 1, c.zz};
      Vec3D ddp{1, 2, az};
      INFO("z=" << z << ", zz=" << c.zz << ", az=" << az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various COM acceleration, COM height = 1.5 and ZMP height = 0.5") {
    double z = 1.5;
    double zz = 0.5;
    struct testcaset_t {
      double az;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, 1.0 + G}, {1.5, 1.5 + G}, {-1, -1.0 + G}, {G, 2.0 * G}};
    for (const auto& c : testcases) {
      Vec3D p{1, 0.1, z};
      Vec3D pz{0, 0.1, zz};
      Vec3D ddp{1, 0.1, c.az};
      INFO("z=" << z << ", zz=" << zz << ", az=" << c.az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("return 0 when (z - zz) <= 0") {
    double az = 0;
    struct testcase_t {
      double z;
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1, 0}, {1, 2, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, c.z};
      Vec3D pz{0, 0, c.zz};
      Vec3D ddp{0, 0, az};
      INFO("z=" << c.z << ", zz=" << c.zz << ", az=" << az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when (az + g) <= 0") {
    double z = 1;
    double zz = 0;
    struct testcase_t {
      double az;
      double expected_sqr_zeta;
    } testcases[] = {{-2.0 * G, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, z};
      Vec3D pz{0, 0, zz};
      Vec3D ddp{0, 0, c.az};
      INFO("z=" << z << ", zz=" << zz << ", az=" << c.az);
      CHECK(computeSqrZeta(p, pz, ddp) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, ddp) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }
}

TEST_CASE("computeSqrZeta(Vec3D&,Vec3D&,Vec3D&,Vec3D&)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  SECTION("various COM height, fixed ZMP = 0, Fz = G, m = 1") {
    double zz = 0;
    double fz = G;
    double m = 1;
    struct testcaset_t {
      double z;
      double expected_sqr_zeta;
    } testcases[] = {{1, G}, {G, 1.0}, {2, G / 2}, {4, G / 4}};
    for (const auto& c : testcases) {
      Vec3D p{1, 2, c.z};
      Vec3D pz{0, 1, zz};
      Vec3D f{1, 0, fz};
      INFO("z=" << c.z << ", zz=" << zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various ZMP height, fixed COM = 2, Fz = G, m = 1") {
    double z = 2;
    double fz = G;
    double m = 1;
    struct testcaset_t {
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {
        {1, G}, {1.5, 2.0 * G}, {0.5, 2.0 * G / 3.0}, {-0.5, 2.0 * G / 5.0}};
    for (const auto& c : testcases) {
      Vec3D p{0, 1, z};
      Vec3D pz{1, 1, c.zz};
      Vec3D f{0, 2, fz};
      INFO("z=" << z << ", zz=" << c.zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various Fz, fixed COM = 2, ZMP = 1, m = 1") {
    double z = 2;
    double zz = 1;
    double m = 1;
    struct testcaset_t {
      double fz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1}, {1.5, 1.5}, {0.5, 0.5}, {G, G}};
    for (const auto& c : testcases) {
      Vec3D p{-1, 0, z};
      Vec3D pz{0.1, 0, zz};
      Vec3D f{-0.1, 0, c.fz};
      INFO("z=" << z << ", zz=" << zz << ", fz=" << c.fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("various mass, fixed COM = 2, ZMP = 1, Fz = 2") {
    double z = 2;
    double zz = 1;
    double fz = 2;
    struct testcaset_t {
      double m;
      double expected_sqr_zeta;
    } testcases[] = {{1, 2}, {2, 1}, {0.5, 4}, {1.5, 4.0 / 3}};
    for (const auto& c : testcases) {
      Vec3D p{1, -1, z};
      Vec3D pz{-1, 1, zz};
      Vec3D f{0, -1, fz};
      INFO("z=" << z << ", zz=" << zz << ", fz=" << fz << ", m=" << c.m);
      CHECK(computeSqrZeta(p, pz, f, c.m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, c.m) == sqrt(c.expected_sqr_zeta));
    }
  }

  SECTION("return 0 when (z - zz) <= 0") {
    double fz = 1;
    double m = 1;
    struct testcase_t {
      double z;
      double zz;
      double expected_sqr_zeta;
    } testcases[] = {{1, 1, 0}, {1, 2, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, c.z};
      Vec3D pz{0, 0, c.zz};
      Vec3D f{0, 0, fz};
      INFO("z=" << c.z << ", zz=" << c.zz << ", fz=" << fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when fz < 0") {
    double z = 2;
    double zz = 1;
    double m = 1;
    struct testcase_t {
      double fz;
      double expected_sqr_zeta;
    } testcases[] = {{-1, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, z};
      Vec3D pz{0, 0, zz};
      Vec3D f{0, 0, c.fz};
      INFO("z=" << z << ", zz=" << zz << ", fz=" << c.fz << ", m=" << m);
      CHECK(computeSqrZeta(p, pz, f, m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }

  SECTION("return 0 when mass < 0") {
    double z = 2;
    double zz = 1;
    double fz = 1;
    struct testcase_t {
      double m;
      double expected_sqr_zeta;
    } testcases[] = {{-1, 0}};
    zEchoOff();
    for (const auto& c : testcases) {
      Vec3D p{0, 0, z};
      Vec3D pz{0, 0, zz};
      Vec3D f{0, 0, fz};
      INFO("z=" << z << ", zz=" << zz << ", fz=" << fz << ", m=" << c.m);
      CHECK(computeSqrZeta(p, pz, f, c.m) == c.expected_sqr_zeta);
      CHECK(computeZeta(p, pz, f, c.m) == sqrt(c.expected_sqr_zeta));
    }
    zEchoOn();
  }
}

TEST_CASE("computeReactForce(Vec3D&,double)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  SECTION("case: various COM acceleration with fixed mass = 1") {
    double mass = 1;
    struct testcase_t {
      Vec3D acc;
      Vec3D expected_force;
    } testcases[] = {{{0, 0, -G}, {0, 0, 0}},
                     {{1, 2, 3}, {1, 2, 3 + G}},
                     {{-1, -3, G}, {-1, -3, 2 * G}}};
    for (const auto& c : testcases) {
      INFO("m = " << mass << ", acc = " << c.acc);
      CHECK(computeReactForce(c.acc, mass) == c.expected_force);
    }
  }

  SECTION("case: various mass with fixed acceleration = (-1, 1, -0.5g)") {
    Vec3D acc(-1, 1, -0.5 * G);
    struct testcase_t {
      double mass;
      Vec3D expected_force;
    } testcases[] = {
        {1, {-1, 1, 0.5 * G}}, {3, {-3, 3, 1.5 * G}}, {5, {-5, 5, 2.5 * G}}};
    for (const auto& c : testcases) {
      INFO("m = " << c.mass << ", acc = " << acc);
      CHECK(computeReactForce(acc, c.mass) == c.expected_force);
    }
  }
}

TEST_CASE("computeReactForce(Vec3D&,Vec3D&,double,double)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  SECTION("mass = 1, zeta2 = G, various COM / ZMP position") {
    double mass = 1;
    double zeta2 = G;
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      Vec3D expected_force;
    } testcases[] = {{{0, 0, 1}, {0, 0, 0}, {0, 0, G}},
                     {{1, 2, 2}, {-1, -1, 1}, {2 * G, 3 * G, G}},
                     {{-1, 0.5, 1}, {1, 0.5, 0}, {-2 * G, 0, G}}};
    for (const auto& c : testcases) {
      INFO("m = " << mass << ", zeta2 = " << zeta2 << ", com = " << c.com
                  << ", zmp = " << c.zmp);
      CHECK(computeReactForce(c.com, c.zmp, zeta2, mass) == c.expected_force);
    }
  }

  SECTION("COM = (1, -1, 1), ZMP = (0, 0, 0), various mass and zeta2") {
    Vec3D com(1, -1, 1);
    Vec3D zmp(0, 0, 0);
    struct testcase_t {
      double mass;
      double zeta2;
      Vec3D expected_force;
    } testcases[] = {{1, 2, {2, -2, 2}},
                     {0.5, 1, {0.5, -0.5, 0.5}},
                     {3, G, {3 * G, -3 * G, 3 * G}}};
    for (const auto& c : testcases) {
      INFO("m = " << c.mass << ", zeta2 = " << c.zeta2 << ", com = " << com
                  << ", zmp = " << zmp);
      CHECK(computeReactForce(com, zmp, c.zeta2, c.mass) == c.expected_force);
    }
  }
}

TEST_CASE("computeReactForce(Vec3D&,Vec3D&,Vec3D&,double)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  struct testcase_t {
    Vec3D com_p;
    Vec3D zmp_p;
    Vec3D com_a;
    double mass;
    Vec3D expected_force;
  } testcases[] = {{{0, 0, 1}, {0, 0, 0}, {0, 0, 0}, 1, {0, 0, G}},
                   {{1, 2, 2}, {0, -1, 0}, {1, 0, G}, 1, {G, 3. * G, 2. * G}},
                   {{-1, 0.5, 2},
                    {1, 1, 0.5},
                    {0, 0, -0.5 * G},
                    2,
                    {-4. * G / 3, -G / 3, G}}};
  for (const auto& c : testcases) {
    INFO("m = " << c.mass << ", com = " << c.com_p << ", zmp = " << c.zmp_p
                << ", com acc = " << c.com_a);
    CHECK(computeReactForce(c.com_p, c.zmp_p, c.com_a, c.mass) ==
          c.expected_force);
  }
}

TEST_CASE("computeReactForce(Vec3D&,Vec3D&,double)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  struct testcase_t {
    Vec3D com_p;
    Vec3D zmp_p;
    double fz;
    Vec3D expected_force;
  } testcases[] = {{{0, 0, 1}, {0, 0, 0}, 1, {0, 0, 1}},
                   {{0, 0, 1}, {0, 0, 0}, 1.5, {0, 0, 1.5}},
                   {{1, -1, 2}, {0, 1, 0.5}, 1.1, {2.2 / 3, -4.4 / 3, 1.1}},
                   {{1, -1, 2}, {0, 1, 0.5}, 30, {20, -40, 30}},
                   {{-0.5, 1.9, 1.4}, {1, -1.1, -0.1}, 1, {-1, 2, 1}},
                   {{-0.5, 1.9, 1.4}, {1, -1.1, -0.1}, 10, {-10, 20, 10}}};
  for (const auto& c : testcases) {
    INFO("com = " << c.com_p << ", zmp = " << c.zmp_p << ", fz = " << c.fz);
    CHECK(computeReactForce(c.com_p, c.zmp_p, c.fz) == c.expected_force);
  }
}

TEST_CASE("computeComAcc(Vec3D&,double)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  SECTION("without additional force") {
    struct testcase_t {
      Vec3D f;
      double m;
      Vec3D expected_acc;
    } testcases[] = {{{1, 1, 0}, 1, {1, 1, -G}},
                     {{-1, 2, G}, 2, {-0.5, 1, -0.5 * G}},
                     {{0, -1, -0.5 * G}, 1.5, {0, -2. / 3, -4. * G / 3}}};
    for (const auto& c : testcases) {
      INFO("m = " << c.m << ", f = " << c.f);
      CHECK(computeComAcc(c.f, c.m) == c.expected_acc);
    }
  }
  SECTION("with additional force") {
    struct testcase_t {
      Vec3D f;
      double m;
      Vec3D ef;
      Vec3D expected_acc;
    } testcases[] = {
        {{1, 1, 0}, 1, {0, 1, 1}, {1, 2, -G + 1}},
        {{-1, 2, G}, 2, {-2, 2, 2 * G}, {-1.5, 2, 0.5 * G}},
        {{0, -1, -0.5 * G}, 1.5, {0.5, -0.5, 0.5 * G}, {1. / 3, -1, -G}}};
    for (const auto& c : testcases) {
      INFO("m = " << c.m << ", f = " << c.f << ", added f = " << c.ef);
      CHECK(computeComAcc(c.f, c.m, c.ef) == c.expected_acc);
    }
  }
}

TEST_CASE("computeComAcc(Vec3D&,Vec3D&,double)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  SECTION("without additional force") {
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      double zeta2;
      Vec3D expected_acc;
    } testcases[] = {{{0, 0, 1}, {0, 0, 0}, G, {0, 0, 0}},
                     {{1, 2, 2.5}, {-1, 1, 0.5}, 2, {4, 2, 4. - G}},
                     {{-1, 1.5, 2}, {0.5, 1.5, 0}, 1.5, {-2.25, 0, 3. - G}}};
    for (const auto& c : testcases) {
      INFO("zeta2 = " << c.zeta2 << ", com = " << c.com << ", zmp = " << c.zmp);
      CHECK(computeComAcc(c.com, c.zmp, c.zeta2) == c.expected_acc);
    }
  }
  SECTION("with additional force") {
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      double zeta2;
      double m;
      Vec3D ef;
      Vec3D expected_acc;
    } testcases[] = {
        {{0, 0, 1}, {0, 0, 0}, G, 1, {1, 1, 1}, {1, 1, 1}},
        {{1, 2, 2.5}, {-1, 1, 0.5}, 2, 1.5, {1.5, -1.5, 3}, {5, 1, 6. - G}},
        {{-1, 1.5, 2}, {0.5, 1.5, 0}, 1.5, 1, {-1, 2, 3}, {-3.25, 2, 6. - G}}};
    for (const auto& c : testcases) {
      INFO("zeta2 = " << c.zeta2 << ", com = " << c.com << ", zmp = " << c.zmp
                      << ", mass = " << c.m << ", ef = " << c.ef);
      CHECK(computeComAcc(c.com, c.zmp, c.zeta2, c.m, c.ef) == c.expected_acc);
    }
  }
}

TEST_CASE("computeComAcc(Vec3D&,Vec3D&,Vec3D&,double)",
          "[corelib][humanoid][com_zmp_model_formula]") {
  SECTION("without additional force") {
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      double m;
      Vec3D f;
      Vec3D expected_acc;
    } testcases[] = {
        {{0, 0, 1}, {0, 0, 0}, 1, {0, 0, G}, {0, 0, 0}},
        {{1, 2, 2.5},
         {-1, 0, 0.5},
         1.5,
         {0, 0, 1},
         {2. / 3, 2. / 3, 2. / 3 - G}},
        {{-1, 1.5, 2}, {0.5, -1.5, 0}, 2, {0, 0, 2}, {-3. / 4, 3. / 2, 1 - G}}};
    for (const auto& c : testcases) {
      INFO("com = " << c.com << ", zmp = " << c.zmp << ", mass = " << c.m
                    << ", f = " << c.f);
      CHECK(computeComAcc(c.com, c.zmp, c.f, c.m) == c.expected_acc);
    }
  }
  SECTION("with additional force") {
    struct testcase_t {
      Vec3D com;
      Vec3D zmp;
      double m;
      Vec3D f;
      Vec3D ef;
      Vec3D expected_acc;
    } testcases[] = {{{0, 0, 1}, {0, 0, 0}, 1, {0, 0, G}, {1, 2, 3}, {1, 2, 3}},
                     {{1, 2, 2.5},
                      {-1, 0, 0.5},
                      1.5,
                      {0, 0, 1},
                      {0.5, -0.5, 0.5},
                      {1, 1. / 3, 1 - G}},
                     {{-1, 1.5, 2},
                      {0.5, -1.5, 0},
                      2,
                      {0, 0, 2},
                      {-1, 1, G},
                      {-5. / 4, 2, 1 - 0.5 * G}}};
    for (const auto& c : testcases) {
      INFO("com = " << c.com << ", zmp = " << c.zmp << ", mass = " << c.m
                    << ", f = " << c.f << ", ef = " << c.ef);
      CHECK(computeComAcc(c.com, c.zmp, c.f, c.m, c.ef) == c.expected_acc);
    }
  }
}

TEST_CASE("computeComAcc(Vec3D&,Vec3D&,Vec3D&,double) 2",
          "[corelib][humanoid][com_zmp_model_formula]") {
  Vec3D force = Vec3D(0, 0, G);
  double mass = 1;

  SECTION("case: the COM height is assumed to be const, namely zeta is const") {
    struct testcase_t {
      Vec3D com_pos;
      Vec3D zmp_pos;
      Vec3D expected_acc;
    } testcases[] = {
        // cases where the COM height equals to G, namely zeta equals to 1.
        {{0, 0, G}, {0, 0, 0}, {0, 0, 0}},
        {{1, 0, G}, {0, 0, 0}, {1, 0, 0}},
        {{3, 0, G}, {0, 0, 0}, {3, 0, 0}},
        {{0, 0, G}, {1, 0, 0}, {-1, 0, 0}},
        {{0, 0, G}, {3, 0, 0}, {-3, 0, 0}},
        {{0, 2, G}, {0, 0, 0}, {0, 2, 0}},
        {{0, 4, G}, {0, 0, 0}, {0, 4, 0}},
        {{0, 0, G}, {0, 2, 0}, {0, -2, 0}},
        {{0, 0, G}, {0, 4, 0}, {0, -4, 0}},
        {{3, 1, G}, {2, 2, 0}, {1, -1, 0}},
        {{1, 3, G}, {-1, 2, 0}, {2, 1, 0}},
        // cases where the COM height equals to 1
        {{0, 0, 1}, {0, 0, 0}, {0, 0, 0}},
        {{1, 0, 1}, {0, 0, 0}, {G, 0, 0}},
        {{3, 0, 1}, {0, 0, 0}, {3 * G, 0, 0}},
        {{0, 0, 1}, {1, 0, 0}, {-G, 0, 0}},
        {{0, 0, 1}, {3, 0, 0}, {-3 * G, 0, 0}},
        {{0, 2, 1}, {0, 0, 0}, {0, 2 * G, 0}},
        {{0, 4, 1}, {0, 0, 0}, {0, 4 * G, 0}},
        {{0, 0, 1}, {0, 2, 0}, {0, -2 * G, 0}},
        {{0, 0, 1}, {0, 4, 0}, {0, -4 * G, 0}},
        {{3, 1, 1}, {2, 2, 0}, {G, -G, 0}},
        {{1, 3, 1}, {-1, 2, 0}, {2 * G, G, 0}},
        // given random values
        {{2, 3, 2}, {-2, -1, 0}, {2 * G, 2 * G, 0}},
        {{1, 3, 0.5}, {-1, -1, 0}, {4 * G, 8 * G, 0}},
    };

    for (auto c : testcases) {
      Vec3D acc = computeComAcc(c.com_pos, c.zmp_pos, force, mass);
      CAPTURE(&c.com_pos);
      CAPTURE(&c.zmp_pos);
      CHECK(acc == c.expected_acc);
    }
  }
}

}  // namespace
}  // namespace holon
