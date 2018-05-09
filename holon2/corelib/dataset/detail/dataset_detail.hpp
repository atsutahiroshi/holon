#ifndef HOLON_DATASET_DETAIL_DATASET_DETAIL_HPP_
#define HOLON_DATASET_DETAIL_DATASET_DETAIL_HPP_

#include <type_traits>
#include <utility>

namespace holon {

template <typename... Ts>
class Dataset;

template <typename... Ts>
void copyDataset(const Dataset<Ts...>& t_src, Dataset<Ts...> t_dst);

namespace dataset_detail {

template <std::size_t I, typename... Ts>
typename std::enable_if<(I == sizeof...(Ts)), void>::type copyDataset_impl(
    Dataset<Ts...>, Dataset<Ts...>) {}

template <std::size_t I, typename... Ts>
typename std::enable_if<(I < sizeof...(Ts)), void>::type copyDataset_impl(
    Dataset<Ts...> t_src, Dataset<Ts...> t_dst) {
  t_dst.template get<I>() = t_src.template get<I>();
  copyDataset_impl<I + 1, Ts...>(t_src, t_dst);
}

template <std::size_t I, typename Tuple, typename... Unpacked>
struct DatasetFromTuple_impl {
  using DataUnit = typename std::tuple_element<I - 1, Tuple>::type;
  using type =
      typename DatasetFromTuple_impl<I - 1, Tuple, DataUnit, Unpacked...>::type;
};

template <typename Tuple, typename... Unpacked>
struct DatasetFromTuple_impl<0, Tuple, Unpacked...> {
  using type = Dataset<Unpacked...>;
};

}  // namespace dataset_detail
}  // namespace holon

#endif  // HOLON_DATASET_DETAIL_DATASET_DETAIL_HPP_
