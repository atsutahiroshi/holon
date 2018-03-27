/* data_set_base - Base class for data set class
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

#ifndef HOLON_DATA_DATA_SET_BASE_HPP_
#define HOLON_DATA_DATA_SET_BASE_HPP_

#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>
#include "holon/corelib/common/utility.hpp"

namespace holon {

// non-member functions
template <typename RawData, typename... Args>
std::shared_ptr<RawData> alloc_raw_data(Args&&... args) {
  return std::make_shared<RawData>(std::forward<Args>(args)...);
}

template <typename T, typename = int>
struct is_data_type : std::false_type {};

template <typename T>
struct is_data_type<T, decltype((void)T::is_data_type, 0)> : std::true_type {};

template <typename Data, typename... Args>
Data make_data(Args&&... args) {
  static_assert(is_data_type<Data>::value,
                "make_data is able to make instance derived from DataSetBase.");
  return Data(std::forward<Args>(args)...);
}

template <template <typename> class Data, typename State, typename... Args>
Data<State> make_data(Args&&... args) {
  static_assert(is_data_type<Data<State>>::value,
                "make_data is able to make instance derived from DataSetBase.");
  return Data<State>(std::forward<Args>(args)...);
}

template <template <typename> class Data, typename State, typename... Args>
Data<State> make_data(const State& state, Args&&... args) {
  static_assert(is_data_type<Data<State>>::value,
                "make_data is able to make instance derived from DataSetBase.");
  return Data<State>(state, std::forward<Args>(args)...);
}

template <class DataType, class... RawDataTypes>
class DataSetBase {
  static_assert(sizeof...(RawDataTypes) > 0,
                "DataSetBase must have at least one RawData class.");
  // static_assert(std::is_default_constructible<DataType>::value,
  //               "DataType must be default constructible");

  using Self = DataSetBase<DataType, RawDataTypes...>;
  static constexpr std::size_t raw_data_num = sizeof...(RawDataTypes);
  static constexpr bool more_than_one = raw_data_num - 1;

 protected:
  using RawDataTuple = std::tuple<RawDataTypes...>;
  using RawDataPtrTuple = std::tuple<std::shared_ptr<RawDataTypes>...>;

 public:
  template <std::size_t I>
  using RawDataI = typename std::tuple_element<I, RawDataTuple>::type;
  template <std::size_t I>
  using RawDataPtrI = typename std::tuple_element<I, RawDataPtrTuple>::type;
  static constexpr bool is_data_type = true;
  static const make_index_seq<raw_data_num> index;

 public:
  DataSetBase()
      : m_data_ptr_tuple(std::make_tuple(alloc_raw_data<RawDataTypes>()...)) {}
  explicit DataSetBase(const RawDataTypes&... args)
      : m_data_ptr_tuple(
            std::make_tuple(alloc_raw_data<RawDataTypes>(args)...)) {}
  explicit DataSetBase(std::shared_ptr<RawDataTypes>... args)
      : m_data_ptr_tuple(std::make_tuple(args...)) {}
  explicit DataSetBase(RawDataPtrTuple t_data_ptr_tuple)
      : m_data_ptr_tuple(t_data_ptr_tuple) {}

  const RawDataPtrTuple& get_data_ptr_tuple() const { return m_data_ptr_tuple; }

  template <std::size_t I = 0>
  const RawDataPtrI<I>& get_ptr() const {
    return std::get<I>(m_data_ptr_tuple);
  }
  template <std::size_t I = 0>
  RawDataPtrI<I>& get_ptr() {
    return std::get<I>(m_data_ptr_tuple);
  }

  template <std::size_t... I>
  auto extract_ptr_tuple() const -> const std::tuple<RawDataPtrI<I>...> {
    return std::make_tuple(std::get<I>(m_data_ptr_tuple)...);
  }
  template <std::size_t... I>
  auto extract_ptr_tuple() -> std::tuple<RawDataPtrI<I>...> {
    return std::make_tuple(std::get<I>(m_data_ptr_tuple)...);
  }
  template <std::size_t... I>
  auto extract_ptr_tuple(index_seq<I...>) const
      -> decltype(this->extract_ptr_tuple<I...>()) {
    return this->extract_ptr_tuple<I...>();
  }
  template <std::size_t... I>
  auto extract_ptr_tuple(index_seq<I...>)
      -> decltype(this->extract_ptr_tuple<I...>()) {
    return this->extract_ptr_tuple<I...>();
  }

  template <std::size_t I = 0>
  const RawDataI<I>& get() const {
    return *std::get<I>(m_data_ptr_tuple);
  }
  template <std::size_t I = 0>
  RawDataI<I>& get() {
    return *std::get<I>(m_data_ptr_tuple);
  }

  using CallOpRetType =
      typename std::conditional<more_than_one, DataType, RawDataI<0>>::type;
  const CallOpRetType& operator()() const {
    return this->call_op_impl(std::integral_constant<bool, more_than_one>());
  }
  CallOpRetType& operator()() {
    return this->call_op_impl(std::integral_constant<bool, more_than_one>());
  }

  template <std::size_t I = 0>
  void copy_index(const Self& other) {
    this->get<I>() = other.get<I>();
  }
  template <std::size_t I = 0>
  void copy_index(const RawDataI<I>& t_raw_data) {
    this->get<I>() = t_raw_data;
  }

  template <typename Data, typename SrcIndex, typename DstIndex>
  void copy_subdata(const Data& data, SrcIndex src_index, DstIndex dst_index) {
    auto src = data.extract_ptr_tuple(src_index);
    auto dst = this->extract_ptr_tuple(dst_index);
    static_assert(std::is_same<decltype(dst), decltype(src)>::value,
                  "copy_data failed. Inappropriate data index selection.");
    copy_data_tuple(this->extract_ptr_tuple(dst_index),
                    data.extract_ptr_tuple(src_index));
  }

  void copy(const Self& other) { copy_impl<0>(other); }
  void copy(const RawDataTypes... args) { copy_data_impl<0>(args...); }
  void copy(const std::shared_ptr<RawDataTypes>... args) {
    copy_data_impl<0>(args...);
  }

  void share_with(const Self& target) {
    m_data_ptr_tuple = target.get_data_ptr_tuple();
  }

  DataType clone() {
    DataType tmp;
    tmp.copy(*this);
    return tmp;
  }

  template <typename SubDataType, std::size_t... I>
  SubDataType extract() {
    // consider using make_data method...
    return SubDataType(std::get<I>(m_data_ptr_tuple)...);
  }

  template <typename SubDataType, std::size_t... I>
  SubDataType extract(index_seq<I...>) {
    // consider using make_data method...
    return SubDataType(std::get<I>(m_data_ptr_tuple)...);
  }

  bool operator==(const Self& rhs) const {
    return m_data_ptr_tuple == rhs.get_data_ptr_tuple();
  }
  bool operator!=(const Self& rhs) const { return !(*this == rhs); }

 private:
  RawDataPtrTuple m_data_ptr_tuple;

  const DataType& call_op_impl(std::true_type) const {
    return static_cast<DataType&>(*this);
  }
  DataType& call_op_impl(std::true_type) {
    return static_cast<DataType&>(*this);
  }

  const RawDataI<0>& call_op_impl(std::false_type) const {
    return this->get<0>();
  }
  RawDataI<0>& call_op_impl(std::false_type) { return this->get<0>(); }

  template <std::size_t I = 0>
  typename std::enable_if<(I == raw_data_num), void>::type copy_impl(
      const Self& /* other */) {}

  template <std::size_t I = 0>
  typename std::enable_if<(I < raw_data_num), void>::type copy_impl(
      const Self& other) {
    copy_index<I>(other);
    copy_impl<I + 1>(other);
  }

  template <std::size_t I>
  void copy_data_impl() {}

  template <std::size_t I, typename T, typename... Types>
  void copy_data_impl(const T& data, Types... args) {
    copy_index<I>(data);
    copy_data_impl<I + 1>(args...);
  }

  template <std::size_t I, typename T, typename... Types>
  void copy_data_impl(std::shared_ptr<T> data_ptr, Types... args) {
    copy_index<I>(*data_ptr);
    copy_data_impl<I + 1>(args...);
  }

  template <typename... Ts>
  void copy_data_tuple(std::tuple<Ts...> src, std::tuple<Ts...> dst) {
    copy_data_tuple_impl<0, Ts...>(src, dst);
  }

  template <std::size_t I, typename... Ts>
  typename std::enable_if<(I == sizeof...(Ts)), void>::type
      copy_data_tuple_impl(std::tuple<Ts...>, std::tuple<Ts...>) {}

  template <std::size_t I, typename... Ts>
  typename std::enable_if<(I < sizeof...(Ts)), void>::type copy_data_tuple_impl(
      std::tuple<Ts...> src, std::tuple<Ts...> dst) {
    *std::get<I>(dst) = *std::get<I>(src);
    copy_data_tuple_impl<I + 1, Ts...>(src, dst);
  }
};

}  // namespace holon

#endif  // HOLON_DATA_DATA_SET_BASE_HPP_
