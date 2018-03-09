/* zip - simple zip function
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

#include "holon/corelib/common/zip.hpp"

#include "catch.hpp"

namespace holon {

TEST_CASE("Check zip function", "[zip]") {
  SECTION("With two arrays") {
    std::array<int, 3> a = {{1, 2, 3}};
    std::array<char, 3> b = {{'a', 'b', 'c'}};
    int cnt = 0;
    for (const auto& i : zip(a, b)) {
      CHECK(std::get<0>(i) == 1 + cnt);
      CHECK(std::get<1>(i) == 'a' + cnt);
      cnt++;
    }
  }
  SECTION("With three arrays") {
    std::array<int, 3> a = {{1, 2, 3}};
    std::array<char, 3> b = {{'a', 'b', 'c'}};
    std::array<double, 3> c = {{2.0, 3.0, 4.0}};
    int cnt = 0;
    for (const auto& i : zip(a, b, c)) {
      CHECK(std::get<0>(i) == 1 + cnt);
      CHECK(std::get<1>(i) == 'a' + cnt);
      CHECK(std::get<2>(i) == 2.0 + cnt);
      cnt++;
    }
  }
}

TEST_CASE("Check zip_ptr function", "[zip_ptr]") {
  SECTION("Assignment") {
    std::array<int, 3> a = {{1, 2, 3}};
    std::array<int, 3> b = {{4, 5, 6}};
    std::array<int, 3> c;
    for (auto&& i : zip_ptr(a, b, c)) {
      *std::get<2>(i) = *std::get<0>(i) + *std::get<1>(i);
    }
    CHECK(c[0] == 5);
    CHECK(c[1] == 7);
    CHECK(c[2] == 9);
  }

  SECTION("Assignment with double") {
    std::array<double, 3> a = {{1.0, 2.0, 3.0}};
    std::array<double, 3> b = {{4.0, 5.0, 6.0}};
    std::array<double, 3> c;
    for (auto&& i : zip_ptr(a, b, c)) {
      *std::get<2>(i) = *std::get<0>(i) + *std::get<1>(i);
    }
    CHECK(c[0] == Approx(5.0));
    CHECK(c[1] == Approx(7.0));
    CHECK(c[2] == Approx(9.0));
  }
}

}  // namespace holon
