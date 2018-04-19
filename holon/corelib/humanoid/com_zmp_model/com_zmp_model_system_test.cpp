/* com_zmp_model_system - Definition of the COM-ZMP model system
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

#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_system.hpp"

#include <roki/rk_g.h>
#include "holon/corelib/humanoid/com_zmp_model/com_zmp_model_formula.hpp"

#include "catch.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace {

namespace cz = com_zmp_model_formula;

const double G = RK_G;

// ComZmpModelSystem class
TEST_CASE("ComZmpModelSystem: constructor", "[ComZmpModelSystem][ctor]") {
  SECTION("constructor with data pointer") {
    auto data = make_data<ComZmpModelData>();
    ComZmpModelSystem sys(data);
    CHECK(sys.data() == data);
  }
}

TEST_CASE("ComZmpModelSystem: accessors / mutators",
          "[ComZmpModelSystem][accessor][mutator]") {
  auto data = make_data<ComZmpModelData>();
  ComZmpModelSystem sys(data);

  SECTION("data pointer") {
    auto data2 = make_data<ComZmpModelData>();
    sys.set_data(data2);
    CHECK(sys.data() == data2);
  }
}

TEST_CASE("ComZmpModelSystem::operator() defines system of COM-ZMP model",
          "[ComZmpModelSystem]") {
  Fuzzer fuzz;
  auto data = make_data<ComZmpModelData>();
  ComZmpModelSystem sys(data);
  Vec3D p = {0, 1, 2};
  Vec3D v = {1, -2, -1};
  std::array<Vec3D, 2> x{{p, v}};
  std::array<Vec3D, 2> dxdt;
  double t = 0;

  SECTION("When function to compute acceleration is specified, use this") {
    auto a = fuzz.get<Vec3D>();
    auto f_acc = [a](const Vec3D&, const Vec3D&, const double) { return a; };
    sys.set_com_acceleration_f(f_acc);
    dxdt = sys(x, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == a);
  }

  SECTION("When function to compute reaction force is specified, use this") {
    auto f = fuzz.get<Vec3D>();
    auto f_force = [f](const Vec3D&, const Vec3D&, const double) { return f; };
    sys.set_reaction_force_f(f_force);
    dxdt = sys(x, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == f - Vec3D(0, 0, G));  // \ddot{p} = \frac{f}{m} - g

    SECTION("External force is also specified, use this as well") {
      auto ef = fuzz.get<Vec3D>();
      auto f_ef = [ef](const Vec3D&, const Vec3D&, const double) { return ef; };
      sys.set_external_force_f(f_ef);
      dxdt = sys(x, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] ==
            f + ef - Vec3D(0, 0, G));  // \ddot{p} = \frac{f + f_{e}}{m} - g
    }
  }

  SECTION("When external force is specified and not reaction force") {
    auto ef = fuzz.get<Vec3D>();
    auto f_ef = [ef](const Vec3D&, const Vec3D&, const double) { return ef; };
    sys.set_external_force_f(f_ef);
    dxdt = sys(x, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == ef);  // \ddot{p} = \frac{mg + f_{e}}{m} - g
  }

  SECTION("When ZMP position is specified, calculate acceleration from it") {
    Vec3D zmp = {1, -1, 0};
    auto f_zmp = [zmp](const Vec3D&, const Vec3D&, const double) {
      return zmp;
    };
    auto expected = cz::com_acceleration(
        p, zmp, Vec3D(0, 0, data.get().mass * G), data.get().mass);
    sys.set_zmp_position_f(f_zmp);
    dxdt = sys(x, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == expected);
  }

  SECTION("When ZMP position and rection force are specified") {
    Vec3D zmp = {1, -1, 0};
    auto f_zmp = [zmp](const Vec3D&, const Vec3D&, const double) {
      return zmp;
    };
    Vec3D fz = {0, 0, 8};
    auto f_fz = [fz](const Vec3D&, const Vec3D&, const double) { return fz; };
    auto expected = cz::com_acceleration(p, zmp, fz, data.get().mass);
    SECTION("Specify fz first then ZMP") {
      sys.set_reaction_force_f(f_fz);
      sys.set_zmp_position_f(f_zmp);
      dxdt = sys(x, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
    SECTION("Specify ZMP first then fz") {
      sys.set_zmp_position_f(f_zmp);
      sys.set_reaction_force_f(f_fz);
      dxdt = sys(x, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
  }

  SECTION("ZMP position, rection force and external force are specified") {
    Vec3D zmp = {1, -1, 0};
    auto f_zmp = [zmp](const Vec3D&, const Vec3D&, const double) {
      return zmp;
    };
    Vec3D fz = {0, 0, 8};
    auto f_fz = [fz](const Vec3D&, const Vec3D&, const double) { return fz; };
    Vec3D ef = fuzz.get<Vec3D>();
    auto f_ef = [ef](const Vec3D&, const Vec3D&, const double) { return ef; };
    auto expected = cz::com_acceleration(p, zmp, fz, data.get().mass, ef);
    SECTION("Specify them in the following order: fz, ZMP, ef") {
      sys.set_reaction_force_f(f_fz);
      sys.set_zmp_position_f(f_zmp);
      sys.set_external_force_f(f_ef);
      dxdt = sys(x, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
    SECTION("Specify them in the following order: ZMP, fz, ef") {
      sys.set_zmp_position_f(f_zmp);
      sys.set_reaction_force_f(f_fz);
      sys.set_external_force_f(f_ef);
      dxdt = sys(x, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
    SECTION("Specify them in the following order: fz, ef, ZMP") {
      sys.set_reaction_force_f(f_fz);
      sys.set_external_force_f(f_ef);
      sys.set_zmp_position_f(f_zmp);
      dxdt = sys(x, t);
      CHECK(dxdt[0] == v);
      CHECK(dxdt[1] == expected);
    }
  }

  SECTION("Mass value is not 1") {
    data.get().mass = 2.5;
    Vec3D zmp = {1, -1, 0};
    auto f_zmp = [zmp](const Vec3D&, const Vec3D&, const double) {
      return zmp;
    };
    Vec3D ef = fuzz.get<Vec3D>();
    auto f_ef = [ef](const Vec3D&, const Vec3D&, const double) { return ef; };
    auto expected = cz::com_acceleration(
        p, zmp, Vec3D(0, 0, data.get().mass * G), data.get().mass, ef);
    sys.set_zmp_position_f(f_zmp);
    sys.set_external_force_f(f_ef);
    dxdt = sys(x, t);
    CHECK(dxdt[0] == v);
    CHECK(dxdt[1] == expected);
  }
}

TEST_CASE("ComZmpModelSystem::is_set_zmp_position",
          "[ComZmpModelSystem][is_set_zmp_position]") {
  auto data = make_data<ComZmpModelData>();
  ComZmpModelSystem sys(data);
  CHECK_FALSE(sys.is_set_zmp_position());
  sys.set_zmp_position_f(
      [](const Vec3D&, const Vec3D&, const double) { return kVec3DZero; });
  CHECK(sys.is_set_zmp_position());
}

}  // namespace
}  // namespace holon
