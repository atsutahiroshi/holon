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

template <typename T, typename Engine, typename Distribution>
struct RandomAdapter {};

template <typename Engine, typename Distribution>
struct RandomAdapter<double, Engine, Distribution> {
  using type = double;
  type get(Engine& eng, Distribution& dist) { return dist(eng); }
};

template <typename T, typename Float = double>
class Random {
  static constexpr Float default_min = -1;
  static constexpr Float default_max = 1;

 public:
  Random()
      : m_rd(),
        m_seed({m_rd(), m_rd(), m_rd(), m_rd(), m_rd()}),
        m_engine(m_seed),
        m_distribution(default_min, default_max),
        m_adapter() {}
  Random(std::seed_seq seed)
      : m_rd(),
        m_seed(seed),
        m_engine(m_seed),
        m_distribution(default_min, default_max),
        m_adapter() {}
  Random(Float min, Float max)
      : m_rd(),
        m_seed({m_rd(), m_rd(), m_rd(), m_rd(), m_rd()}),
        m_engine(m_seed),
        m_distribution(min, max),
        m_adapter() {}

  // member functions
  T operator()() { return this->get(); }
  // T get() { return m_distribution(m_engine); }
  T get() { return m_adapter.get(m_engine, m_distribution); }

 private:
  std::random_device m_rd;
  std::seed_seq m_seed;
  std::mt19937 m_engine;
  std::uniform_real_distribution<Float> m_distribution;
  RandomAdapter<T, std::mt19937, std::uniform_real_distribution<Float>>
      m_adapter;
};

}  // namespace holon

#endif  // HOLON_COMMON_RANDOM_HPP_
