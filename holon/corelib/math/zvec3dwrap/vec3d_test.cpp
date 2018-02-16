/* vec3d - 3D vector that wraps zVec3D
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

#include "holon/corelib/math/zvec3dwrap/vec3d.hpp"

#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include "catch.hpp"
#include "holon/test/util/catch/custom_matchers.hpp"
#include "holon/test/util/fuzzer/fuzzer.hpp"

namespace holon {
namespace zVec3DWrap {
namespace {

using Catch::Matchers::Equals;

TEST_CASE("zVec3DWrap::Vec3D: constructors", "[corelib][math][Vec3D]") {
  Fuzzer fuzz;

  SECTION("default constructor should initialize with zeros") {
    Vec3D v;
    CHECK(v[0] == 0.0);
    CHECK(v[1] == 0.0);
    CHECK(v[2] == 0.0);
  }

  SECTION("constructor with three arguments") {
    double x = fuzz.get();
    double y = fuzz.get();
    double z = fuzz.get();
    Vec3D v(x, y, z);
    CHECK(v[0] == x);
    CHECK(v[1] == y);
    CHECK(v[2] == z);
  }
}

TEST_CASE("zVec3DWrap::Vec3D: copy constructor", "[corelib][math][Vec3D]") {
  Vec3D a(1.0, 2.0, 3.0);
  Vec3D b(a);
  CHECK_THAT(b, Equals(a));
}

TEST_CASE("zVec3DWrap::Vec3D: copy assignment operator",
          "[corelib][math][Vec3D]") {
  Fuzzer fuzz;
  Vec3D a, b;
  a[0] = fuzz.get();
  a[1] = fuzz.get();
  a[2] = fuzz.get();
  b = a;
  CHECK_THAT(b, Equals(a));
}

TEST_CASE("zVec3DWrap::Vec3D: move constructor", "[corelib][math][Vec3D]") {
  Vec3D a1(2, 3, 4);
  Vec3D a2(2, 3, 4);

  Vec3D b = std::move(a1);
  CHECK_THAT(b, Equals(a1));

  auto f = [](Vec3D arg) { return arg; };
  Vec3D c = f(Vec3D(2, 3, 4));
  CHECK_THAT(c, Equals(a2));
}

TEST_CASE("zVec3DWrap::Vec3D: move assignment operator",
          "[corelib][math][Vec3D]") {
  Vec3D a1(3, 4, 5);
  Vec3D a2(3, 4, 5);
  Vec3D b, c;

  b = std::move(a1);
  CHECK_THAT(b, Equals(a2));

  auto f = [](Vec3D arg) { return arg; };
  c = f(Vec3D(3, 4, 5));
  CHECK_THAT(c, Equals(a2));
}

TEST_CASE("zVec3DWrap::Vec3D: subscript operator", "[corelib][math][Vec3D]") {
  Vec3D a;
  a[0] = 1.0;
  a[1] = 2.0;
  a[2] = 3.0;
  CHECK(a[0] == 1.0);
  CHECK(a[1] == 2.0);
  CHECK(a[2] == 3.0);
}

TEST_CASE("zVec3DWrap::Vec3D: accessors/mutators", "[corelib][math][Vec3D]") {
  SECTION("x, y, z") {
    Vec3D a;
    Fuzzer fuzz;
    double x = fuzz.get();
    double y = fuzz.get();
    double z = fuzz.get();
    a.set_x(x);
    a.set_y(y);
    a.set_z(z);
    CHECK(a.x() == x);
    CHECK(a.y() == y);
    CHECK(a.z() == z);
  }
}

TEST_CASE("zVec3DWrap::Vec3D: function size returns 3",
          "[corelib][math][Vec3D]") {
  Vec3D a;
  CHECK(a.size() == 3);
}

TEST_CASE("zVec3DWrap::Vec3D: clone the object", "[corelib][math][Vec3D]") {
  Vec3D a, b;
  Fuzzer fuzz;
  fuzz.randomize(a);
  b = a.clone();
  CHECK_THAT(b, Equals(a));
}

TEST_CASE("zVec3DWrap::Vec3D: clone the object which has opposite values",
          "[corelib][math][Vec3D]") {
  Vec3D a, b;
  Fuzzer fuzz;
  fuzz.randomize(a);
  b = a.opposite();
  CHECK(b[0] == -a[0]);
  CHECK(b[1] == -a[1]);
  CHECK(b[2] == -a[2]);
}

TEST_CASE("zVec3DWrap::Vec3D: clear the elements", "[corelib][math][Vec3D]") {
  Vec3D a;
  Fuzzer fuzz;
  fuzz.randomize(a);
  a.clear();
  CHECK_THAT(a, Equals(kVec3DZero));
}

TEST_CASE("zVecDWrap::Vec3D: make string") {
  SECTION("case1") {
    Vec3D a(1, 2, 3);
    CHECK_THAT(a.str(), Equals(std::string("( 1, 2, 3 )")));
  }
  SECTION("case2") {
    Vec3D a(0.1, 0.2, 0.3);
    CHECK_THAT(a.str(), Equals(std::string("( 0.1, 0.2, 0.3 )")));
  }
}

TEST_CASE("zVecDWrap::Vec3D: make string for data") {
  SECTION("case1") {
    Vec3D a(1, 2, 3);
    std::string s("1.0000000000e+00 2.0000000000e+00 3.0000000000e+00");
    CHECK_THAT(a.data(), Equals(s));
  }
  SECTION("case2") {
    Vec3D a(0.1, 0.2, 0.3);
    std::string s("1.0000000000e-01 2.0000000000e-01 3.0000000000e-01");
    CHECK_THAT(a.data(), Equals(s));
  }
  SECTION("case3") {
    Vec3D a(10, 20, 30);
    std::string s("1.0000000000e+01, 2.0000000000e+01, 3.0000000000e+01");
    CHECK_THAT(a.data(", "), Equals(s));
  }
  SECTION("case4") {
    Vec3D a(0.1, 0.2, 0.3);
    std::string s("1.0000e-01 2.0000e-01 3.0000e-01");
    CHECK_THAT(a.data(4), Equals(s));
  }
  SECTION("case5") {
    Vec3D a(0.1, 0.2, 0.3);
    std::string s("1.00e-01,2.00e-01,3.00e-01");
    CHECK_THAT(a.data(",", 2), Equals(s));
  }
}

TEST_CASE("zVecDWrap::Vec3D: insertion to stream") {
  SECTION("case1") {
    Vec3D a(1, 2, 3);
    std::stringstream ss;
    ss << a;
    CHECK_THAT(ss.str(), Equals(std::string("( 1, 2, 3 )")));
  }
  SECTION("case2") {
    Vec3D a(0.1, 0.2, 0.3);
    std::stringstream ss;
    ss << a;
    CHECK_THAT(ss.str(), Equals(std::string("( 0.1, 0.2, 0.3 )")));
  }
  SECTION("case3") {
    Vec3D a(0.1, 0.2, 0.3);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4) << a;
    CHECK_THAT(ss.str(), Equals(std::string("( 0.1000, 0.2000, 0.3000 )")));
  }
}

TEST_CASE("zVec3DWrap::Vec3D: unary plus operator", "[corelib][math][Vec3D]") {
  Vec3D a, b;
  Fuzzer fuzz;
  fuzz.randomize(a);
  b = +a;
  CHECK_THAT(b, Equals(a));
}

TEST_CASE("zVec3DWrap::Vec3D: unary minus operator", "[corelib][math][Vec3D]") {
  Vec3D a, b;
  Fuzzer fuzz;
  fuzz.randomize(a);
  b = -a;
  CHECK(b[0] == -a[0]);
  CHECK(b[1] == -a[1]);
  CHECK(b[2] == -a[2]);
}

TEST_CASE("zVec3DWrap::Vec3D: addition", "[corelib][math][Vec3D]") {
  SECTION("add Vec3D object by calling member function") {
    Vec3D a = {1, 2, 3};
    Vec3D b = {4, 5, 6};
    Vec3D c = a.add(b);
    CHECK_THAT(c, Equals(Vec3D(5, 7, 9)));
    a = {.1, .2, .3};
    b = {.4, .5, .6};
    c = a.add(b);
    CHECK_THAT(c, Equals(Vec3D(.5, .7, .9)));
  }

  SECTION("add Vec3D object with operator+") {
    Vec3D a = {1, 2, 3};
    Vec3D b = {4, 5, 6};
    Vec3D c = a + b;
    CHECK_THAT(c, Equals(Vec3D(5, 7, 9)));
    a = {.1, .2, .3};
    b = {.4, .5, .6};
    c = a + b;
    CHECK_THAT(c, Equals(Vec3D(.5, .7, .9)));
  }

  SECTION("add scalar by calling member function") {
    Vec3D a = {1, 2, 3};
    double k = 4;
    Vec3D b = a.add(k);
    CHECK_THAT(b, Equals(Vec3D(5, 6, 7)));
    a = {.1, .2, .3};
    k = .4;
    b = a.add(k);
    CHECK_THAT(b, Equals(Vec3D(.5, .6, .7)));
  }

  SECTION("add scalar with operator+") {
    Vec3D a = {1, 2, 3};
    double k = 4;
    Vec3D b = a + k;
    CHECK_THAT(b, Equals(Vec3D(5, 6, 7)));
    a = {.1, .2, .3};
    k = .4;
    b = a + k;
    CHECK_THAT(b, Equals(Vec3D(.5, .6, .7)));
  }

  SECTION("added by scalar with operator+") {
    Vec3D a = {1, 2, 3};
    double k = 4;
    Vec3D b = k + a;
    CHECK_THAT(b, Equals(Vec3D(5, 6, 7)));
    a = {.1, .2, .3};
    k = .4;
    b = k + a;
    CHECK_THAT(b, Equals(Vec3D(.5, .6, .7)));
  }

  SECTION("consecutive addition of Vec3D objects") {
    Vec3D a = {1, 2, 3};
    Vec3D b = {4, 5, 6};
    Vec3D c = {7, 8, 9};
    Vec3D d = a + b + c;
    CHECK_THAT(d, Equals(Vec3D(12, 15, 18)));
  }

  SECTION("consecutive addition combined with Vec3D and scalar") {
    Vec3D a = {1, 2, 3};
    Vec3D b = {4, 5, 6};
    double k = 7;
    Vec3D c = a + k + b;
    CHECK_THAT(c, Equals(Vec3D(12, 14, 16)));
  }
}

TEST_CASE("zVec3DWrap::Vec3D: subtraction", "[corelib][math][Vec3D]") {
  SECTION("subtract Vec3D object by calling member function") {
    Vec3D a = {1, 2, 3};
    Vec3D b = {4, 5, 6};
    Vec3D c = a.sub(b);
    CHECK_THAT(c, Equals(Vec3D(-3, -3, -3)));
    a = {.1, .2, .3};
    b = {.4, .5, .6};
    c = a.sub(b);
    CHECK_THAT(c, Equals(Vec3D(-.3, -.3, -.3)));
  }

  SECTION("subtract Vec3D object with operator+") {
    Vec3D a = {1, 2, 3};
    Vec3D b = {4, 5, 6};
    Vec3D c = a - b;
    CHECK_THAT(c, Equals(Vec3D(-3, -3, -3)));
    a = {.1, .2, .3};
    b = {.4, .5, .6};
    c = a - b;
    CHECK_THAT(c, Equals(Vec3D(-.3, -.3, -.3)));
  }

  SECTION("subtract scalar by calling member function") {
    Vec3D a = {1, 2, 3};
    double k = 4;
    Vec3D b = a.sub(k);
    CHECK_THAT(b, Equals(Vec3D(-3, -2, -1)));
    a = {.1, .2, .3};
    k = .4;
    b = a.sub(k);
    CHECK_THAT(b, Equals(Vec3D(-.3, -.2, -.1)));
  }

  SECTION("subtract scalar with operator-") {
    Vec3D a = {1, 2, 3};
    double k = 4;
    Vec3D b = a - k;
    CHECK_THAT(b, Equals(Vec3D(-3, -2, -1)));
    a = {.1, .2, .3};
    k = .4;
    b = a - k;
    CHECK_THAT(b, Equals(Vec3D(-.3, -.2, -.1)));
  }

  SECTION("subtracted by scalar with operator-") {
    Vec3D a = {1, 2, 3};
    double k = 4;
    Vec3D b = k - a;
    CHECK_THAT(b, Equals(Vec3D(3, 2, 1)));
    a = {.1, .2, .3};
    k = .4;
    b = k - a;
    CHECK_THAT(b, Equals(Vec3D(.3, .2, .1)));
  }

  SECTION("consecutive subtraction of Vec3D objects") {
    Vec3D a = {1, 2, 3};
    Vec3D b = {4, 5, 6};
    Vec3D c = {7, 8, 9};
    Vec3D d = a - b - c;
    CHECK_THAT(d, Equals(Vec3D(-10, -11, -12)));
  }

  SECTION("consecutive subtraction combined with Vec3D and scalar") {
    Vec3D a = {1, 2, 3};
    Vec3D b = {4, 5, 6};
    double k = 7;
    Vec3D c = a - k - b;
    CHECK_THAT(c, Equals(Vec3D(-10, -10, -10)));
  }
}

TEST_CASE("zVec3DWrap::Vec3D: multiplication", "[corelib][math][Vec3D]") {
  SECTION("multiply scalar by calling member function") {
    Vec3D a = {1, 2, 3};
    double k = 4;
    Vec3D b = a.mul(k);
    CHECK_THAT(b, Equals(Vec3D(4, 8, 12)));
    a = {.1, .2, .3};
    k = .4;
    b = a.mul(k);
    CHECK_THAT(b, Equals(Vec3D(.04, .08, .12)));
  }

  SECTION("multiply scalar with operator*") {
    Vec3D a = {1, 2, 3};
    double k = 4;
    Vec3D b = a * k;
    CHECK_THAT(b, Equals(Vec3D(4, 8, 12)));
    a = {.1, .2, .3};
    k = .4;
    b = a * k;
    CHECK_THAT(b, Equals(Vec3D(.04, .08, .12)));
  }

  SECTION("multiplied by scalar with operator*") {
    Vec3D a = {1, 2, 3};
    double k = 4;
    Vec3D b = k * a;
    CHECK_THAT(b, Equals(Vec3D(4, 8, 12)));
    a = {.1, .2, .3};
    k = .4;
    b = k * a;
    CHECK_THAT(b, Equals(Vec3D(.04, .08, .12)));
  }
}

TEST_CASE("zVec3DWrap::Vec3D: division", "[corelib][math][Vec3D]") {
  SECTION("div scalar by calling member function") {
    Vec3D a = {10, 20, 30};
    double k = 4;
    Vec3D b = a.div(k);
    CHECK_THAT(b, Equals(Vec3D(2.5, 5, 7.5)));
    a = {10, 20, 30};
    k = .4;
    b = a.div(k);
    CHECK_THAT(b, Equals(Vec3D(25, 50, 75)));
  }

  SECTION("div scalar with operator+") {
    Vec3D a = {10, 20, 30};
    double k = 4;
    Vec3D b = a / k;
    CHECK_THAT(b, Equals(Vec3D(2.5, 5, 7.5)));
    a = {10, 20, 30};
    k = .4;
    b = a / k;
    CHECK_THAT(b, Equals(Vec3D(25, 50, 75)));
  }

  SECTION("warn division by zero") {
    zEchoOff();
    Vec3D a = {10, 20, 30};
    double k = 0;
    Vec3D b = a / k;
    CHECK_THAT(b, Equals(a));
    zEchoOn();
  }
}

TEST_CASE("zVec3DWrap::Vec3D: range-based loop", "[corelib][math][Vec3D]") {
  SECTION("check value") {
    Vec3D a = {1, 2, 3};
    int cnt = 0;
    for (auto&& e : a) {
      CHECK(e == ++cnt);
    }
    CHECK(cnt == 3);
  }

  SECTION("modify value") {
    Vec3D a = {1, 2, 3};
    int cnt = 0;
    for (auto&& e : a) {
      e = e + 3;
      CHECK(e == ++cnt + 3);
    }
    CHECK(cnt == 3);
  }
}

TEST_CASE("zVec3DWrap::Vec3D: loop with iterator", "[corelib][math][Vec3D]") {
  SECTION("check value") {
    Vec3D a = {10, 20, 30};
    int cnt = 0;
    for (auto it = a.begin(); it != a.end(); ++it) {
      CHECK(*it == 10.0 * ++cnt);
    }
    CHECK(cnt == 3);
  }

  SECTION("modify value") {
    Vec3D a = {10, 20, 30};
    int cnt = 0;
    for (auto it = a.begin(); it != a.end(); ++it) {
      *it = *it + 10;
      CHECK(*it == 10.0 * ++cnt + 10);
    }
    CHECK(cnt == 3);
  }
}

}  // namespace
}  // namespace zVec3DWrap
}  // namespace holon