/* contact_force_generator - Contact force generator class
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

#include "holon2/corelib/humanoid/com_controller/contact_force_generator.hpp"

#include "third_party/catch/catch.hpp"

namespace holon {
namespace {

using ::Catch::Matchers::ApproxEquals;
const double kTOL = 1e-10;

TEST_CASE("contact_force_generator: check c'tor",
          "[ComController][ContactForceGenerator]") {
  ComControllerData data;
  const ContactForceGenerator cf(data);
  CHECK(cf.data() == data);
}

TEST_CASE("contact_force_generator: set data",
          "[ComController][ContactForceGenerator]") {
  ComControllerData data1;
  ContactForceGenerator cf(data1);
  ComControllerData data2;
  REQUIRE(cf.data() != data2);
  cf.setData(data2);
  CHECK(cf.data() == data2);
}

TEST_CASE("contact_force_generator: accessor to control parameters",
          "[ComController][ContactForceGenerator]") {
  Random<double> rnd(0, 1);
  std::array<double, 3> q1 = {rnd(), rnd(), rnd()};
  ComControllerData data;
  data.get<2>().q1 = q1;
  const ContactForceGenerator cf(data);
  for (auto i = 0; i < 3; ++i) {
    CHECK(cf.params().q1[i] == q1[i]);
  }
}

TEST_CASE("contact_force_generator: calculate desired contact force",
          "[ComController][ContactForceGenerator]") {
  Random<Vec3d> vec(0, 2);
  Random<Array3d> arr(0, 1);
  Random<double> rnd(0, 1);
  Vec3d p = vec(), v = vec();
  ComControllerData data;
  auto& params = data.get<2>();
  params.com_position = vec();
  params.com_velocity = vec();
  params.q1 = arr();
  params.q2 = arr();
  data.get<0>().mass = rnd();
  ContactForceGenerator cf(data);
  auto expected_fz = cf.calculateZ(p.z(), v.z());
  Vec3d expected_cf(0, 0, expected_fz);
  CHECK_THAT(cf.calculate(p, v), ApproxEquals(expected_cf, kTOL));
}

TEST_CASE("contact_force_generator: ",
          "[ComController][ContactForceGenerator]") {}

}  // namespace
}  // namespace holon
