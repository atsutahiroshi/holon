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

#include <memory>
#include <tuple>
#include "holon2/corelib/common/utility.hpp"

namespace holon {

template <typename RawData, typename... Args>
std::shared_ptr<RawData> makeRawDataPtr(Args&&... args) {
  return std::make_shared<RawData>(std::forward<Args>(args)...);
}

template <typename... RawDataTypes>
class Dataset {
 protected:
  static constexpr std::size_t kRawDataNum = sizeof...(RawDataTypes);
  using Self = Dataset<RawDataTypes...>;
  using RawDataTuple = std::tuple<RawDataTypes...>;
  using RawDataPtrTuple = std::tuple<std::shared_ptr<RawDataTypes>...>;

 public:
  template <std::size_t I>
  using RawDataType = typename std::tuple_element<I, RawDataTuple>::type;
  template <std::size_t I>
  using RawDataPtrType = typename std::tuple_element<I, RawDataPtrTuple>::type;

 public:
  Dataset()
      : m_raw_data_ptr_tuple(
            std::make_tuple(makeRawDataPtr<RawDataTypes>()...)) {}
  virtual ~Dataset() = default;

  constexpr std::size_t getRawDataNum() const { return this->kRawDataNum; }

  const RawDataPtrTuple& getRawDataPtrTuple() const {
    return m_raw_data_ptr_tuple;
  }

  template <std::size_t I = 0>
  const RawDataPtrType<I>& getRawDataPtr() const {
    return std::get<I>(this->getRawDataPtrTuple());
  }

  template <std::size_t I = 0>
  const RawDataType<I>& getRawData() const {
    return *this->getRawDataPtr<I>();
  }
  template <std::size_t I = 0>
  RawDataType<I>& getRawData() {
    return *this->getRawDataPtr<I>();
  }

  template <std::size_t... I>
  auto getRawDataPtrSubTuple() const -> const std::tuple<RawDataPtrType<I>...> {
    return std::make_tuple(std::get<I>(m_raw_data_ptr_tuple)...);
  }
  template <std::size_t... I>
  auto getRawDataPtrSubTuple(IndexSeq<I...>)
      -> const std::tuple<RawDataPtrType<I>...> {
    return this->getRawDataPtrSubTuple<I...>();
  }

 private:
  RawDataPtrTuple m_raw_data_ptr_tuple;
};

}  // namespace holon

#endif  // HOLON_DATASET_DATASET_HPP_
