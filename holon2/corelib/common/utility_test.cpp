/* utility - Utility functions and classes for template meta programming
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

#include "holon2/corelib/common/utility.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

TEST_CASE("utility: IndexSeqence represents a sequence of integers",
          "[utility][IndexSeq]") {
  SECTION("case 1") {
    IndexSeq<1, 2, 3> seq;
    CHECK(seq.size() == 3);
    CHECK(seq.get<0>() == 1);
    CHECK(seq.get<1>() == 2);
    CHECK(seq.get<2>() == 3);
  }
  SECTION("case 2") {
    IndexSeq<1, 3, 5, 7> seq;
    CHECK(seq.size() == 4);
    CHECK(seq.get<0>() == 1);
    CHECK(seq.get<1>() == 3);
    CHECK(seq.get<2>() == 5);
    CHECK(seq.get<3>() == 7);
  }
}

TEST_CASE("utility: makeIndexSeq defines consecutive index sequence",
          "[utility][IndexSeq]") {
  SECTION("case 1") {
    auto seq = makeIndexSeq<3>();
    CHECK(seq.size() == 3);
    CHECK(seq.get<0>() == 0);
    CHECK(seq.get<1>() == 1);
    CHECK(seq.get<2>() == 2);
  }
  SECTION("case 2") {
    auto seq = makeIndexSeq<10>();
    CHECK(seq.size() == 10);
    CHECK(seq.get<0>() == 0);
    CHECK(seq.get<1>() == 1);
    CHECK(seq.get<2>() == 2);
    CHECK(seq.get<3>() == 3);
    CHECK(seq.get<4>() == 4);
    CHECK(seq.get<5>() == 5);
    CHECK(seq.get<6>() == 6);
    CHECK(seq.get<7>() == 7);
    CHECK(seq.get<8>() == 8);
    CHECK(seq.get<9>() == 9);
  }
}

}  // namespace
}  // namespace holon
