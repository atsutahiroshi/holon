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

#include <roki/rk_g.h>

#include <sstream>
#include <string>
#include "catch.hpp"

namespace Catch {
namespace Detail {

std::string zVec3DToString(const zVec3D* v) {
  std::ostringstream ss;
  ss << "( ";
  if (!v) {
    ss << "null vector";
  } else {
    ss << zVec3DElem(v, zX) << ", " << zVec3DElem(v, zY) << ", "
       << zVec3DElem(v, zZ);
  }
  ss << " )";
  return ss.str();
}

}  // namespace Detail

template <>
struct StringMaker<zVec3D*> {
  static std::string convert(zVec3D* v) {
    return ::Catch::Detail::zVec3DToString(v);
  }
};

class zVec3DMatcher : public MatcherBase<zVec3D*> {
  const zVec3D* comparator_;

 public:
  explicit zVec3DMatcher(const zVec3D* comparator) : comparator_(comparator) {}

  bool match(zVec3D* v) const override {
    for (std::size_t i = 0; i < 3; ++i)
      if (!zIsTiny(zVec3DElem(comparator_, i) - zVec3DElem(v, i))) return false;
    return true;
  }
  std::string describe() const override {
    std::ostringstream ss;
    return "Equals: " + ::Catch::Detail::zVec3DToString(comparator_);
  }
};

// The builder function
inline zVec3DMatcher Equals(const zVec3D* comparator) {
  return zVec3DMatcher(comparator);
}
}  // namespace Catch

namespace holon {
namespace {

const double G = RK_G;

TEST_CASE("compute zeta squared in equation of motion based on COM-ZMP model",
          "[corelib][humanoid]") {
  ComZmpModel model;

  SECTION("zeta should be computed according to the height of COM") {
    struct testcase_t {
      double height_com;
      double expected_zeta_squared;
      double expected_zeta;
    } testcases[] = {{1, G, sqrt(G)},
                     {G, 1.0, 1.0},
                     {2, G / 2, sqrt(G / 2)},
                     {4, G / 4, sqrt(G / 4)}};

    for (auto c : testcases) {
      zVec3D pg = {0, 0, c.height_com};
      CHECK(model.ComputeZetaSqr(&pg) == Approx(c.expected_zeta_squared));
      CHECK(model.ComputeZeta(&pg) == Approx(c.expected_zeta));
    }
  }
  SECTION("return 0 when the given height of COM was 0") {
    // Returning 0 when the height is 0 is due to avoiding zero-division,
    // but this is not theoritically correct.
    // This case should be handled as an exception somehow.
    // TODO(*): handle zero-division error correctly
    zVec3D pg = {0, 0, 0};
    zEchoOff();
    CHECK_FALSE(isinf(model.ComputeZetaSqr(&pg)));
    CHECK(model.ComputeZetaSqr(&pg) == 0.0);
    CHECK_FALSE(isinf(model.ComputeZeta(&pg)));
    CHECK(model.ComputeZeta(&pg) == 0.0);
    zEchoOn();
  }
  SECTION("return 0 when the given height of COM was negative") {
    // Return 0 when a negative valued was given as the height of COM.
    // This should be handled as an exception as well.
    // TODO(*): handle the case where a negative value is given
    zVec3D pg = {0, 0, -1};
    zEchoOff();
    CHECK(model.ComputeZetaSqr(&pg) == 0.0);
    CHECK_FALSE(isnan(model.ComputeZeta(&pg)));
    CHECK(model.ComputeZeta(&pg) == 0.0);
    zEchoOn();
  }
}

TEST_CASE("compute acceleration of COM based on COM-ZMP model",
          "[corelib][humanoid]") {
  ComZmpModel model;

  SECTION("case: the COM height is assumed to be const, namely zeta is const") {
    struct testcase_t {
      zVec3D pos_com;
      zVec3D pos_zmp;
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
      model.ComputeAcceleration(&c.pos_com, &c.pos_zmp, &acc);
      CAPTURE(&c.pos_com);
      CAPTURE(&c.pos_zmp);
      CHECK_THAT(&acc, Catch::Equals(&c.expected_acc));
    }
  }
}

}  // namespace
}  // namespace holon
