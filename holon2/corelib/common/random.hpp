/* random - Random number generator
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

#ifndef HOLON_COMMON_RANDOM_HPP_
#define HOLON_COMMON_RANDOM_HPP_

#include <algorithm>
#include <array>
#include <random>
#include <type_traits>

namespace holon {

template <typename T, typename Enable = void>
class Random {};

template <typename T>
class Random<T,
             typename std::enable_if<std::is_floating_point<T>::value>::type> {
 public:
  Random()
      : m_rd(),
        m_seed({m_rd(), m_rd(), m_rd(), m_rd(), m_rd()}),
        m_engine(m_seed),
        m_distribution(-1, 1) {}

  // member functions
  T operator()() { return this->get(); }
  T get() { return m_distribution(m_engine); }

 private:
  std::random_device m_rd;
  std::seed_seq m_seed;
  std::mt19937 m_engine;
  std::uniform_real_distribution<T> m_distribution;
};

}  // namespace holon

#endif  // HOLON_COMMON_RANDOM_HPP_
