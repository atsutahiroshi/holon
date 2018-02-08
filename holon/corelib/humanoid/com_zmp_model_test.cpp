/* com_zmp_model - COM-ZMP model
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

#include "holon/corelib/humanoid/com_zmp_model.hpp"

#include <zm/zm_ieee.h>
#include <roki/rk_g.h>

#include "catch.hpp"
#include "holon/test/util/catch/custom_matchers.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

const double G = RK_G;

TEST_CASE("COM-ZMP model has a mass as a parameter", "[corelib][humanoid]") {
  SECTION("when instantiated with no paramters mass should be 1") {
    ComZmpModel model;
    CHECK(model.mass() == 1.0);
  }

  SECTION("can be instantiated by providing mass") {
    {
      ComZmpModel model(1.0);
      CHECK(model.mass() == 1.0);
    }
    {
      ComZmpModel model(3.0);
      CHECK(model.mass() == 3.0);
    }
  }

  SECTION("can provide mass after instantiation") {
    ComZmpModel model;
    REQUIRE(model.mass() == 1.0);

    Fuzzer fuzz;
    for (auto i = 0; i < 3; ++i) {
      double m = fuzz.get();
      model.set_mass(m);
      CHECK(model.mass() == m);
    }
  }

  SECTION("when non-positive value is given as mass it should be 1") {
    ComZmpModel model;

    zEchoOff();
    model.set_mass(3);
    REQUIRE(model.mass() == 3.0);
    model.set_mass(0);
    CHECK(model.mass() == 1.0);

    model.set_mass(3);
    REQUIRE(model.mass() == 3.0);
    model.set_mass(-1);
    CHECK(model.mass() == 1.0);
    zEchoOn();
  }
}

TEST_CASE("check if states in COM-ZMP model are initialized approapriately") {
  ComZmpModel model;

  SECTION("COM position") {
    zVec3D expected_com_pos = {0, 0, 1};
    REQUIRE_THAT(model.com_position(),
                 Catch::Matchers::Equals(&expected_com_pos));
  }
  SECTION("COM velocity") {
    zVec3D expected_com_vel = {0, 0, 0};
    REQUIRE_THAT(model.com_velocity(),
                 Catch::Matchers::Equals(&expected_com_vel));
  }
  SECTION("COM acceleration") {
    zVec3D expected_com_acc = {0, 0, 0};
    REQUIRE_THAT(model.com_acceleration(),
                 Catch::Matchers::Equals(&expected_com_acc));
  }
  SECTION("ZMP position") {
    zVec3D expected_zmp_pos = {0, 0, 0};
    REQUIRE_THAT(model.zmp_position(),
                 Catch::Matchers::Equals(&expected_zmp_pos));
  }
}

TEST_CASE("check if you can set arbitrarly states") {
  ComZmpModel model;
  Fuzzer fuzz;

  SECTION("COM position") {
    zVec3D new_com_pos;
    fuzz.randomize(&new_com_pos);
    REQUIRE_THAT(model.com_position(), Catch::Matchers::NotEqual(&new_com_pos));
    model.set_com_position(&new_com_pos);
    REQUIRE_THAT(model.com_position(), Catch::Matchers::Equals(&new_com_pos));
  }
  SECTION("COM velocity") {
    zVec3D new_com_vel;
    fuzz.randomize(&new_com_vel);
    REQUIRE_THAT(model.com_velocity(), Catch::Matchers::NotEqual(&new_com_vel));
    model.set_com_velocity(&new_com_vel);
    REQUIRE_THAT(model.com_velocity(), Catch::Matchers::Equals(&new_com_vel));
  }
  SECTION("ZMP position") {
    zVec3D new_zmp_pos;
    fuzz.randomize(&new_zmp_pos);
    REQUIRE_THAT(model.zmp_position(), Catch::Matchers::NotEqual(&new_zmp_pos));
    model.set_zmp_position(&new_zmp_pos);
    REQUIRE_THAT(model.zmp_position(), Catch::Matchers::Equals(&new_zmp_pos));
  }
}

TEST_CASE("compute zeta squared in equation of motion based on COM-ZMP model",
          "[corelib][humanoid]") {
  ComZmpModel model;

  SECTION("zeta should be computed according to the COM height") {
    struct testcase_t {
      double com_height;
      double expected_zeta_squared;
      double expected_zeta;
    } testcases[] = {{1, G, sqrt(G)},
                     {G, 1.0, 1.0},
                     {2, G / 2, sqrt(G / 2)},
                     {4, G / 4, sqrt(G / 4)}};

    for (auto c : testcases) {
      zVec3D pg = {0, 0, c.com_height};
      CHECK(model.computeZetaSqr(&pg) == Approx(c.expected_zeta_squared));
      CHECK(model.computeZeta(&pg) == Approx(c.expected_zeta));
    }
  }
  SECTION("return 0 when the given COM height was 0") {
    // Returning 0 when the height is 0 is due to avoiding zero-division,
    // but this is not theoritically correct.
    // This case should be handled as an exception somehow.
    // TODO(*): handle zero-division error correctly
    zVec3D pg = {0, 0, 0};
    zEchoOff();
    CHECK_FALSE(zIsInf(model.computeZetaSqr(&pg)));
    CHECK(model.computeZetaSqr(&pg) == 0.0);
    CHECK_FALSE(zIsInf(model.computeZeta(&pg)));
    CHECK(model.computeZeta(&pg) == 0.0);
    zEchoOn();
  }
  SECTION("return 0 when the given COM height was negative") {
    // Return 0 when a negative valued was given as the height of COM.
    // This should be handled as an exception as well.
    // TODO(*): handle the case where a negative value is given
    zVec3D pg = {0, 0, -1};
    zEchoOff();
    CHECK(model.computeZetaSqr(&pg) == 0.0);
    CHECK_FALSE(zIsNan(model.computeZeta(&pg)));
    CHECK(model.computeZeta(&pg) == 0.0);
    zEchoOn();
  }
}

TEST_CASE("compute the COM acceleration based on COM-ZMP model",
          "[corelib][humanoid]") {
  ComZmpModel model;

  SECTION("case: the COM height is assumed to be const, namely zeta is const") {
    struct testcase_t {
      zVec3D com_pos;
      zVec3D zmp_pos;
      zVec3D expected_acc;
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
      zVec3D acc;
      model.computeAcceleration(&c.com_pos, &c.zmp_pos, &acc);
      CAPTURE(&c.com_pos);
      CAPTURE(&c.zmp_pos);
      CHECK_THAT(&acc, Catch::Matchers::Equals(&c.expected_acc));
    }
  }
}

}  // namespace
}  // namespace holon
