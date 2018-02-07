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

#include "holon/test/util/fuzzer/fuzzer.hpp"

#include <algorithm>
#include <functional>

namespace holon {

Fuzzer::Fuzzer()
    : m_rd(),
      m_seed({m_rd(), m_rd(), m_rd(), m_rd(), m_rd()}),
      m_engine(m_seed) {}
Fuzzer::Fuzzer(const std::seed_seq& seed) : m_seed(seed), m_engine(m_seed) {}
Fuzzer::Fuzzer(int seed) : m_seed({seed}), m_engine(m_seed) {}

Fuzzer::~Fuzzer() = default;

}  // namespace holon
