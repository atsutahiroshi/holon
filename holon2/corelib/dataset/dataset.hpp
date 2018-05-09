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

template <typename DataUnit, typename... Args>
std::shared_ptr<DataUnit> makeDataUnitPtr(Args&&... args) {
  return std::make_shared<DataUnit>(std::forward<Args>(args)...);
}

template <typename... Ts>
void copyDataset(const Dataset<Ts...>& t_src, Dataset<Ts...> t_dst) {
  dataset_detail::copyDataset_impl<0, Ts...>(t_src, t_dst);
}

template <typename... DataUnitTypes>
class Dataset {
 protected:
  static constexpr std::size_t kDataUnitNum = sizeof...(DataUnitTypes);
  using Self = Dataset<DataUnitTypes...>;

 public:
  using DataUnitTuple = std::tuple<DataUnitTypes...>;
  using DataUnitPtrTuple = std::tuple<std::shared_ptr<DataUnitTypes>...>;
  template <std::size_t I>
  using DataUnitType = typename std::tuple_element<I, DataUnitTuple>::type;
  template <std::size_t I>
  using DataUnitPtrType =
      typename std::tuple_element<I, DataUnitPtrTuple>::type;

 public:
  // constructors
  Dataset()
      : m_data_unit_ptr_tuple(
            std::make_tuple(makeDataUnitPtr<DataUnitTypes>()...)) {}
  explicit Dataset(std::shared_ptr<DataUnitTypes>... args)
      : m_data_unit_ptr_tuple(std::make_tuple(args...)) {}
  explicit Dataset(DataUnitPtrTuple t_tuple) : m_data_unit_ptr_tuple(t_tuple) {}
  explicit Dataset(const DataUnitTypes&... args)
      : m_data_unit_ptr_tuple(
            std::make_tuple(makeDataUnitPtr<DataUnitTypes>(args)...)) {}

  // special member functions
  Dataset(const Self&) = default;
  Dataset(Self&&) = default;
  Self& operator=(const Self&) = default;
  Self& operator=(Self&&) = default;
  virtual ~Dataset() = default;

  static constexpr std::size_t size() { return kDataUnitNum; }

  template <std::size_t I = 0>
  const DataUnitPtrType<I>& getptr() const {
    return std::get<I>(this->tuple());
  }

  template <std::size_t I = 0>
  const DataUnitType<I>& get() const {
    return *this->getptr<I>();
  }
  template <std::size_t I = 0>
  DataUnitType<I>& get() {
    return *this->getptr<I>();
  }

  template <std::size_t... I>
  auto subdata() const -> Dataset<DataUnitType<I>...> {
    return Dataset<DataUnitType<I>...>(this->subtuple<I...>());
  }
  template <std::size_t... I>
  auto subdata(IndexSeq<I...>) const -> Dataset<DataUnitType<I>...> {
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
  void copy(const DataUnitType<I>&... t_data_unit) {
    this->copy_impl<I...>(t_data_unit...);
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
  auto clone() const -> Dataset<DataUnitType<I>, DataUnitType<Rest>...> {
    Dataset<DataUnitType<I>, DataUnitType<Rest>...> ret;
    ret.copy(this->subdata<I, Rest...>());
    return ret;
  }

  bool operator==(const Self& rhs) const { return tuple() == rhs.tuple(); }
  bool operator!=(const Self& rhs) const { return !(*this == rhs); }

 protected:
  auto tuple() const -> const DataUnitPtrTuple& {
    return m_data_unit_ptr_tuple;
  }

  template <std::size_t... I>
  auto subtuple() const -> const std::tuple<DataUnitPtrType<I>...> {
    return std::make_tuple(std::get<I>(m_data_unit_ptr_tuple)...);
  }
  template <std::size_t... I>
  auto subtuple(IndexSeq<I...>) -> const std::tuple<DataUnitPtrType<I>...> {
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
  void copy_impl(const DataUnitType<I>& t_data_unit,
                 const DataUnitType<Rest>&... args) {
    this->get<I>() = t_data_unit;
    this->copy_impl<Rest...>(args...);
  }

 private:
  DataUnitPtrTuple m_data_unit_ptr_tuple;

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

// make Dataset from std::tuple
template <typename Tuple>
using DatasetFromTuple = typename dataset_detail::DatasetFromTuple_impl<
    std::tuple_size<Tuple>::value, Tuple>::type;

// concatenate all Dataset in args
template <typename... Datasets>
struct DatasetCatHelper {
  using tuple = decltype(std::tuple_cat(typename Datasets::DataUnitTuple{}...));
  using type = DatasetFromTuple<tuple>;
};
template <typename... Datasets>
using DatasetCat = typename DatasetCatHelper<Datasets...>::type;

}  // namespace holon

#endif  // HOLON_DATASET_DATASET_HPP_
