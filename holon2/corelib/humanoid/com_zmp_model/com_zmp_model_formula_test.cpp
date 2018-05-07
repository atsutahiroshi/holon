/* com_zmp_model_formula - Mathematical formulae related to COM-ZMP model
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

#include "holon2/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

#include <cmath>
#include "holon2/corelib/common/random.hpp"
#include "holon2/corelib/humanoid/const_defs.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace com_zmp_model_formula {
namespace {

using std::sqrt;
const double G = kGravAccel;
using ::Catch::Matchers::ApproxEquals;

namespace zeta_test_1 {

struct testcase_t {
  double z, zz, az;
  double expected_zeta_sqr;
};
testcase_t testcases1[] = {
    {1, 0, 0, G}, {G, 0, 0, 1.0}, {2, 0, 0, G / 2}, {4, 0, 0, G / 4}};
testcase_t testcases2[] = {{2, 1, 0, G},
                           {2, 1.5, 0, 2.0 * G},
                           {2, 0.5, 0, 2.0 * G / 3.0},
                           {2, -0.5, 0, 2.0 * G / 5.0}};
testcase_t testcases3[] = {{1.5, 0.5, 1, 1.0 + G},
                           {1.5, 0.5, 1.5, 1.5 + G},
                           {1.5, 0.5, -1, -1.0 + G},
                           {1.5, 0.5, G, 2.0 * G}};
testcase_t testcases4[] = {{1, 1, 0, 0}, {1, 2, 0, 0}};
testcase_t testcases5[] = {{1, 0, -2.0 * G, 0}};

template <class testcase_t>
void checkZeta_1(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK(zetaSqr(tc.z, tc.zz, tc.az) == tc.expected_zeta_sqr);
    CHECK(zeta(tc.z, tc.zz, tc.az) == sqrt(tc.expected_zeta_sqr));
  }
}
template <class testcase_t>
void checkZeta_2(const testcase_t& testcases) {
  Random<double> rnd(0, 2);
  for (const auto& tc : testcases) {
    Vec3d p(rnd(), rnd(), tc.z);
    Vec3d pz(rnd(), rnd(), tc.zz);
    Vec3d ddp(rnd(), rnd(), tc.az);
    CHECK(zetaSqr(p, pz, ddp) == tc.expected_zeta_sqr);
    CHECK(zeta(p, pz, ddp) == sqrt(tc.expected_zeta_sqr));
  }
}
TEST_CASE("com_zmp_model_formula: compute zeta from COM accel.: case 1",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases1); }
  SECTION("overloaded function 2") { checkZeta_2(testcases1); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from COM accel.: case 2",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases2); }
  SECTION("overloaded function 2") { checkZeta_2(testcases2); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from COM accel.: case 3",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases3); }
  SECTION("overloaded function 2") { checkZeta_2(testcases3); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from COM accel.: case 4",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases4); }
  SECTION("overloaded function 2") { checkZeta_2(testcases4); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from COM accel.: case 5",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases5); }
  SECTION("overloaded function 2") { checkZeta_2(testcases5); }
}

}  // namespace zeta_test_1

namespace zeta_test_2 {

struct testcase_t {
  double z, zz, fz, m;
  double expected_zeta_sqr;
};
testcase_t testcases1[] = {{1, 0, G, 1, G},
                           {G, 0, G, 1, 1.0},
                           {2, 0, G, 1, G / 2},
                           {4, 0, G, 1, G / 4}};
testcase_t testcases2[] = {{2, 1, G, 1, G},
                           {2, 1.5, G, 1, 2.0 * G},
                           {2, 0.5, G, 1, 2.0 * G / 3.0},
                           {2, -0.5, G, 1, 2.0 * G / 5.0}};
testcase_t testcases3[] = {
    {2, 1, 1, 1, 1}, {2, 1, 1.5, 1.5, 1}, {2, 1, 0.5, 0.5, 1}, {2, 1, G, G, 1}};
testcase_t testcases4[] = {{2, 1, 2, 1, 2},
                           {2, 1, 2, 2, 1},
                           {2, 1, 2, 0.5, 4},
                           {2, 1, 2, 1.5, 4.0 / 3}};
testcase_t testcases5[] = {{1, 1, 1, 1, 0}, {1, 2, 1, 1, 0}};
testcase_t testcases6[] = {{2, 1, -1, 1, 0}};
testcase_t testcases7[] = {{2, 1, 1, -1, 0}};

template <class testcase_t>
void checkZeta_1(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK(zetaSqr(tc.z, tc.zz, tc.fz, tc.m) == tc.expected_zeta_sqr);
    CHECK(zeta(tc.z, tc.zz, tc.fz, tc.m) == sqrt(tc.expected_zeta_sqr));
  }
}
template <class testcase_t>
void checkZeta_2(const testcase_t& testcases) {
  Random<double> rnd(0, 2);
  for (const auto& tc : testcases) {
    Vec3d p(rnd(), rnd(), tc.z);
    Vec3d pz(rnd(), rnd(), tc.zz);
    Vec3d fz(rnd(), rnd(), tc.fz);
    CHECK(zetaSqr(p, pz, fz, tc.m) == tc.expected_zeta_sqr);
    CHECK(zeta(p, pz, fz, tc.m) == sqrt(tc.expected_zeta_sqr));
  }
}
TEST_CASE("com_zmp_model_formula: compute zeta from reaction force: case 1",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases1); }
  SECTION("overloaded function 2") { checkZeta_2(testcases1); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from reaction force: case 2",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases2); }
  SECTION("overloaded function 2") { checkZeta_2(testcases2); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from reaction force: case 3",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases3); }
  SECTION("overloaded function 2") { checkZeta_2(testcases3); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from reaction force: case 4",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases4); }
  SECTION("overloaded function 2") { checkZeta_2(testcases4); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from reaction force: case 5",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases5); }
  SECTION("overloaded function 2") { checkZeta_2(testcases5); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from reaction force: case 6",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases6); }
  SECTION("overloaded function 2") { checkZeta_2(testcases6); }
}
TEST_CASE("com_zmp_model_formula: compute zeta from reaction force: case 7",
          "[com_zmp_model_formula][zeta][zetaSqr]") {
  SECTION("overloaded function 1") { checkZeta_1(testcases7); }
  SECTION("overloaded function 2") { checkZeta_2(testcases7); }
}

}  // namespace zeta_test_2

namespace reaction_force_1 {
struct testcase_t {
  Vec3d ddp;
  double mass;
  Vec3d expected_f;
};
testcase_t testcases1[] = {{{0, 0, -G}, 1, {0, 0, 0}},
                           {{1, 2, 3}, 1, {1, 2, 3 + G}},
                           {{-1, -3, G}, 1, {-1, -3, 2 * G}}};
testcase_t testcases2[] = {{{-1, 1, -0.5 * G}, 1, {-1, 1, 0.5 * G}},
                           {{-1, 1, -0.5 * G}, 3, {-3, 3, 1.5 * G}},
                           {{-1, 1, -0.5 * G}, 5, {-5, 5, 2.5 * G}}};
template <class testcase_t>
void checkReactForce(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(reactForce(tc.ddp, tc.mass), ApproxEquals(tc.expected_f));
  }
}
TEST_CASE(
    "com_zmp_model_formula: compute reaction force from COM accel: case 1",
    "[com_zmp_model_formula][reactForce]") {
  checkReactForce(testcases1);
}
TEST_CASE(
    "com_zmp_model_formula: compute reaction force from COM accel: case 2",
    "[com_zmp_model_formula][reactForce]") {
  checkReactForce(testcases2);
}
}  // namespace reaction_force_1

namespace reaction_force_2 {
struct testcase_t {
  Vec3d p;
  Vec3d pz;
  double zeta2;
  double mass;
  Vec3d expected_f;
};
testcase_t testcases1[] = {{{0, 0, 1}, {0, 0, 0}, G, 1, {0, 0, G}},
                           {{1, 2, 2}, {-1, -1, 1}, G, 1, {2 * G, 3 * G, G}},
                           {{-1, 0.5, 1}, {1, 0.5, 0}, G, 1, {-2 * G, 0, G}}};

testcase_t testcases2[] = {
    {{1, -1, 1}, {0, 0, 0}, 1, 2, {2, -2, 2}},
    {{1, -1, 1}, {0, 0, 0}, 0.5, 1, {0.5, -0.5, 0.5}},
    {{1, -1, 1}, {0, 0, 0}, 3, G, {3 * G, -3 * G, 3 * G}}};
template <class testcase_t>
void checkReactForce(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(reactForce(tc.p, tc.pz, tc.zeta2, tc.mass),
               ApproxEquals(tc.expected_f));
  }
}
TEST_CASE(
    "com_zmp_model_formula: compute reaction force from COM/ZMP, zeta: case 1",
    "[com_zmp_model_formula][reactForce]") {
  checkReactForce(testcases1);
}
TEST_CASE(
    "com_zmp_model_formula: compute reaction force from COM/ZMP, zeta: case 2",
    "[com_zmp_model_formula][reactForce]") {
  checkReactForce(testcases2);
}
}  // namespace reaction_force_2

namespace reaction_force_3 {
struct testcase_t {
  Vec3d p;
  Vec3d pz;
  Vec3d ddp;
  double mass;
  Vec3d expected_f;
};
testcase_t testcases1[] = {
    {{0, 0, 1}, {0, 0, 0}, {0, 0, 0}, 1, {0, 0, G}},
    {{1, 2, 2}, {0, -1, 0}, {1, 0, G}, 1, {G, 3. * G, 2. * G}},
    {{-1, 0.5, 2}, {1, 1, 0.5}, {0, 0, -0.5 * G}, 2, {-4. * G / 3, -G / 3, G}}};
template <class testcase_t>
void checkReactForce(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(reactForce(tc.p, tc.pz, tc.ddp, tc.mass),
               ApproxEquals(tc.expected_f));
  }
}
TEST_CASE(
    "com_zmp_model_formula: compute reaction force from COM/ZMP, accel: case 1",
    "[com_zmp_model_formula][reactForce]") {
  checkReactForce(testcases1);
}
}  // namespace reaction_force_3

namespace reaction_force_4 {
struct testcase_t {
  Vec3d p;
  Vec3d pz;
  double fz;
  Vec3d expected_f;
};
testcase_t testcases1[] = {
    {{0, 0, 1}, {0, 0, 0}, 1, {0, 0, 1}},
    {{0, 0, 1}, {0, 0, 0}, 1.5, {0, 0, 1.5}},
    {{1, -1, 2}, {0, 1, 0.5}, 1.1, {2.2 / 3, -4.4 / 3, 1.1}},
    {{1, -1, 2}, {0, 1, 0.5}, 30, {20, -40, 30}},
    {{-0.5, 1.9, 1.4}, {1, -1.1, -0.1}, 1, {-1, 2, 1}},
    {{-0.5, 1.9, 1.4}, {1, -1.1, -0.1}, 10, {-10, 20, 10}}};
template <class testcase_t>
void checkReactForce(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(reactForce(tc.p, tc.pz, tc.fz), ApproxEquals(tc.expected_f));
  }
}
TEST_CASE(
    "com_zmp_model_formula: compute reaction force from vertical force: case 1",
    "[com_zmp_model_formula][reactForce]") {
  checkReactForce(testcases1);
}
}  // namespace reaction_force_4

namespace com_acceleration_1 {
struct testcase_t {
  Vec3d f;
  double mass;
  Vec3d ef;
  Vec3d expected_ddp;
};
testcase_t testcases1[] = {
    {{1, 1, 0}, 1, {0, 0, 0}, {1, 1, -G}},
    {{-1, 2, G}, 2, {0, 0, 0}, {-0.5, 1, -0.5 * G}},
    {{0, -1, -0.5 * G}, 1.5, {0, 0, 0}, {0, -2. / 3, -4. * G / 3}}};
testcase_t testcases2[] = {
    {{1, 1, 0}, 1, {0, 1, 1}, {1, 2, -G + 1}},
    {{-1, 2, G}, 2, {-2, 2, 2 * G}, {-1.5, 2, 0.5 * G}},
    {{0, -1, -0.5 * G}, 1.5, {0.5, -0.5, 0.5 * G}, {1. / 3, -1, -G}}};
template <class testcase_t>
void checkComAccel_1(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(comAccel(tc.f, tc.mass), ApproxEquals(tc.expected_ddp));
  }
}
template <class testcase_t>
void checkComAccel_2(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(comAccel(tc.f, tc.mass, tc.ef), ApproxEquals(tc.expected_ddp));
  }
}
TEST_CASE("com_zmp_model_formula: compute COM accel. from react force: case 1",
          "[com_zmp_model_formula][comAccel]") {
  checkComAccel_1(testcases1);
}
TEST_CASE("com_zmp_model_formula: compute COM accel. from react force: case 2",
          "[com_zmp_model_formula][comAccel]") {
  checkComAccel_2(testcases2);
}
}  // namespace com_acceleration_1

namespace com_acceleration_2 {
struct testcase_t {
  Vec3d p;
  Vec3d pz;
  double zeta2;
  double mass;
  Vec3d ef;
  Vec3d expected_ddp;
};
testcase_t testcases1[] = {
    {{0, 0, 1}, {0, 0, 0}, G, 1, {0, 0, 0}, {0, 0, 0}},
    {{1, 2, 2.5}, {-1, 1, 0.5}, 2, 1, {0, 0, 0}, {4, 2, 4. - G}},
    {{-1, 1.5, 2}, {0.5, 1.5, 0}, 1.5, 1, {0, 0, 0}, {-2.25, 0, 3. - G}}};
testcase_t testcases2[] = {
    {{0, 0, 1}, {0, 0, 0}, G, 1, {1, 1, 1}, {1, 1, 1}},
    {{1, 2, 2.5}, {-1, 1, 0.5}, 2, 1.5, {1.5, -1.5, 3}, {5, 1, 6. - G}},
    {{-1, 1.5, 2}, {0.5, 1.5, 0}, 1.5, 1, {-1, 2, 3}, {-3.25, 2, 6. - G}}};
template <class testcase_t>
void checkComAccel_1(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(comAccel(tc.p, tc.pz, tc.zeta2), ApproxEquals(tc.expected_ddp));
  }
}
template <class testcase_t>
void checkComAccel_2(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(comAccel(tc.p, tc.pz, tc.zeta2, tc.mass, tc.ef),
               ApproxEquals(tc.expected_ddp));
  }
}
TEST_CASE(
    "com_zmp_model_formula: compute COM accel. from COM/ZMP and zeta: case 1",
    "[com_zmp_model_formula][comAccel]") {
  checkComAccel_1(testcases1);
}
TEST_CASE(
    "com_zmp_model_formula: compute COM accel. from COM/ZMP and zeta: case 2",
    "[com_zmp_model_formula][comAccel]") {
  checkComAccel_2(testcases2);
}
}  // namespace com_acceleration_2

namespace com_acceleration_3 {
struct testcase_t {
  Vec3d p;
  Vec3d pz;
  Vec3d f;
  double mass;
  Vec3d ef;
  Vec3d expected_ddp;
};
testcase_t testcases1[] = {
    {{0, 0, 1}, {0, 0, 0}, {0, 0, G}, 1, {0, 0, 0}, {0, 0, 0}},
    {{1, 2, 2.5},
     {-1, 0, 0.5},
     {0, 0, 1},
     1.5,
     {0, 0, 0},
     {2. / 3, 2. / 3, 2. / 3 - G}},
    {{-1, 1.5, 2},
     {0.5, -1.5, 0},
     {0, 0, 2},
     2,
     {0, 0, 0},
     {-3. / 4, 3. / 2, 1 - G}}};
testcase_t testcases2[] = {
    {{0, 0, 1}, {0, 0, 0}, {0, 0, G}, 1, {1, 2, 3}, {1, 2, 3}},
    {{1, 2, 2.5},
     {-1, 0, 0.5},
     {0, 0, 1},
     1.5,
     {0.5, -0.5, 0.5},
     {1, 1. / 3, 1 - G}},
    {{-1, 1.5, 2},
     {0.5, -1.5, 0},
     {0, 0, 2},
     2,
     {-1, 1, G},
     {-5. / 4, 2, 1 - 0.5 * G}}};
template <class testcase_t>
void checkComAccel_1(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(comAccel(tc.p, tc.pz, tc.f, tc.mass),
               ApproxEquals(tc.expected_ddp));
  }
}
template <class testcase_t>
void checkComAccel_2(const testcase_t& testcases) {
  for (const auto& tc : testcases) {
    CHECK_THAT(comAccel(tc.p, tc.pz, tc.f, tc.mass, tc.ef),
               ApproxEquals(tc.expected_ddp));
  }
}
TEST_CASE(
    "com_zmp_model_formula: compute COM accel. from COM/ZMP and F: case 1",
    "[com_zmp_model_formula][comAccel]") {
  checkComAccel_1(testcases1);
}
TEST_CASE(
    "com_zmp_model_formula: compute COM accel. from COM/ZMP and F: case 2",
    "[com_zmp_model_formula][comAccel]") {
  checkComAccel_2(testcases2);
}
}  // namespace com_acceleration_3

}  // namespace
}  // namespace com_zmp_model_formula
}  // namespace holon
