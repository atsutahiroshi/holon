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
  t_dst.template getRawData<I>() = t_src.template getRawData<I>();
  copyDataset_impl<I + 1, Ts...>(t_src, t_dst);
}

}  // namespace dataset_detail
}  // namespace holon

#endif  // HOLON_DATASET_DETAIL_DATASET_DETAIL_HPP_
