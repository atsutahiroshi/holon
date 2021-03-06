/* fuzzer - simple fuzzer
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

#ifndef HOLON_TEST_UTIL_FUZZER_FUZZER_HPP_
#define HOLON_TEST_UTIL_FUZZER_FUZZER_HPP_

#include <zeo/zeo_vec3d.h>
#include <random>
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

class Fuzzer {
  const double default_min = -10;
  const double default_max = 10;

 public:
  // constructors
  Fuzzer();
  Fuzzer(double t_min, double t_max);
  Fuzzer(const std::seed_seq& t_seed);
  Fuzzer(const std::seed_seq& t_seed, double t_min, double t_max);

  // special member functions
  virtual ~Fuzzer() = default;
  Fuzzer(const Fuzzer&) = delete;
  Fuzzer(Fuzzer&&) = delete;
  Fuzzer& operator=(const Fuzzer&) = delete;
  Fuzzer& operator=(Fuzzer&&) = delete;

  // functions
  double operator()() { return m_distribution(m_engine); }
  template <typename T = double>
  T get() {
    return T((*this)());
  }
  zVec3D randomize(zVec3D& v);
  Vec3D randomize(Vec3D& v);

 private:
  std::random_device m_rd;
  std::seed_seq m_seed;
  std::mt19937 m_engine;
  std::uniform_real_distribution<double> m_distribution;
};

template <>
Vec3D Fuzzer::get<Vec3D>();

}  // namespace holon

#endif  // HOLON_TEST_UTIL_FUZZER_FUZZER_HPP_
