/* dataset - Dataset class
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

#ifndef HOLON_DATASET_DATASET_HPP_
#define HOLON_DATASET_DATASET_HPP_

// template <typename... RawDataTypes>
template <typename RawDataTypes>
class Dataset {
 public:
  RawDataTypes& get() { return m_rawdata; }

 private:
  RawDataTypes m_rawdata;
};

#endif  // HOLON_DATASET_DATASET_HPP_
