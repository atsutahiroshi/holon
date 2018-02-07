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

#include <random>

namespace holon {

class Fuzzer {
 public:
  Fuzzer();
  Fuzzer(const std::seed_seq& seed);
  Fuzzer(int seed);
  virtual ~Fuzzer();

  inline int get() { return m_engine(); };

 private:
  std::random_device m_rd;
  std::seed_seq m_seed;
  std::mt19937 m_engine;
};

}  // namespace holon

#endif  // HOLON_TEST_UTIL_FUZZER_FUZZER_HPP_
