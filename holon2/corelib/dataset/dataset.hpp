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
#include "holon2/corelib/dataset/detail/dataset_detail.hpp"

namespace holon {

template <typename RawData, typename... Args>
std::shared_ptr<RawData> makeRawDataPtr(Args&&... args) {
  return std::make_shared<RawData>(std::forward<Args>(args)...);
}

template <typename... Ts>
void copyDataset(const Dataset<Ts...>& t_src, Dataset<Ts...> t_dst) {
  dataset_detail::copyDataset_impl<0, Ts...>(t_src, t_dst);
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
  Dataset(std::shared_ptr<RawDataTypes>... args)
      : m_raw_data_ptr_tuple(std::make_tuple(args...)) {}
  Dataset(RawDataPtrTuple t_tuple) : m_raw_data_ptr_tuple(t_tuple) {}
  virtual ~Dataset() = default;

  constexpr std::size_t size() const { return this->kRawDataNum; }

  template <std::size_t I = 0>
  const RawDataPtrType<I>& getptr() const {
    return std::get<I>(this->tuple());
  }

  template <std::size_t I = 0>
  const RawDataType<I>& get() const {
    return *this->getptr<I>();
  }
  template <std::size_t I = 0>
  RawDataType<I>& get() {
    return *this->getptr<I>();
  }

  template <std::size_t... I>
  auto subdata() const -> Dataset<RawDataType<I>...> {
    return Dataset<RawDataType<I>...>(this->subtuple<I...>());
  }
  template <std::size_t... I>
  auto subdata(IndexSeq<I...>) const -> Dataset<RawDataType<I>...> {
    return this->subdata<I...>();
  }

  template <typename T = void>
  void copy(const Self& t_other) {
    copyDataset(t_other, *this);
  }
  template <std::size_t I, std::size_t... Rest>
  void copy(const Self& t_other) {
    this->copy_impl<I, Rest...>(t_other);
  }
  template <std::size_t... I>
  void copy(const RawDataType<I>&... t_raw_data) {
    this->copy_impl<I...>(t_raw_data...);
  }
  template <typename... Ts, std::size_t... SrcIdx, std::size_t... DstIdx>
  void copy(const Dataset<Ts...>& t_src, IndexSeq<SrcIdx...> t_src_idx,
            IndexSeq<DstIdx...> t_dst_idx) {
    auto src = t_src.subdata(t_src_idx);
    auto dst = this->subdata(t_dst_idx);
    copyDataset(src, dst);
  }

  template <typename T = void>
  Self clone() const {
    Self ret;
    ret.copy(*this);
    return ret;
  }
  template <std::size_t I, std::size_t... Rest>
  auto clone() const -> Dataset<RawDataType<I>, RawDataType<Rest>...> {
    Dataset<RawDataType<I>, RawDataType<Rest>...> ret;
    ret.copy(this->subdata<I, Rest...>());
    return ret;
  }

 protected:
  auto tuple() const -> const RawDataPtrTuple& { return m_raw_data_ptr_tuple; }

  template <std::size_t... I>
  auto subtuple() const -> const std::tuple<RawDataPtrType<I>...> {
    return std::make_tuple(std::get<I>(m_raw_data_ptr_tuple)...);
  }
  template <std::size_t... I>
  auto subtuple(IndexSeq<I...>) -> const std::tuple<RawDataPtrType<I>...> {
    return this->subtuple<I...>();
  }

 private:
  template <typename T = void>
  void copy_impl(const Self&) {}
  template <std::size_t I, std::size_t... Rest>
  void copy_impl(const Self& t_other) {
    this->get<I>() = t_other.get<I>();
    this->copy_impl<Rest...>(t_other);
  }

  template <typename T = void>
  void copy_impl() {}
  template <std::size_t I, std::size_t... Rest>
  void copy_impl(const RawDataType<I>& t_raw_data,
                 const RawDataType<Rest>&... args) {
    this->get<I>() = t_raw_data;
    this->copy_impl<Rest...>(args...);
  }

 private:
  RawDataPtrTuple m_raw_data_ptr_tuple;

  template <typename... Ts>
  friend std::ostream& operator<<(std::ostream& os, const Dataset<Ts...>& data);
};

// stream insertion
template <typename... Ts>
std::ostream& operator<<(std::ostream& os, const Dataset<Ts...>& data) {
  os << "{";
  printTuple(os, data.tuple(), makeIndexSeq<sizeof...(Ts)>());
  return os << "}";
}

}  // namespace holon

#endif  // HOLON_DATASET_DATASET_HPP_
