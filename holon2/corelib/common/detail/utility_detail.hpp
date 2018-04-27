#ifndef HOLON_COMMON_DETAIL_UTILITY_DETAIL_HPP_
#define HOLON_COMMON_DETAIL_UTILITY_DETAIL_HPP_

#include <type_traits>
#include <utility>

namespace holon {

// forward declaration
template <std::size_t... Ints>
struct IndexSeq;

namespace utility_detail {

// implementation of makeIndexSeq
template <std::size_t N, typename T>
struct makeIndexSeq_impl;

template <std::size_t N, std::size_t... I>
struct makeIndexSeq_impl<N, IndexSeq<I...>> {
  using type = typename makeIndexSeq_impl<N - 1, IndexSeq<N - 1, I...>>::type;
};

template <std::size_t... I>
struct makeIndexSeq_impl<0, IndexSeq<I...>> {
  using type = IndexSeq<I...>;
};

}  // namespace utility_detail
}  // namespace holon

#endif  // HOLON_COMMON_DETAIL_UTILITY_DETAIL_HPP_
