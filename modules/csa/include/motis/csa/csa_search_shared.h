#pragma once

#include <algorithm>
#include <array>
#include <limits>

#include "motis/core/schedule/time.h"

namespace motis {
namespace csa {

constexpr duration MAX_TRAVEL_TIME = 1440;
constexpr duration MAX_TRANSFERS = 7;

template <search_dir Dir, typename TimeType>
constexpr auto INVALID_TIME = Dir == search_dir::FWD
                                  ? std::numeric_limits<TimeType>::max()
                                  : std::numeric_limits<TimeType>::min();

template <typename Cont, typename T, typename Comp = std::less<T>>
auto find_item_location(Cont const& cont, T const& item, Comp comp = Comp()) {
  // find first element not less than item
  return std::lower_bound(cont.begin(), cont.end(), item, comp);
}

template <typename Cont, typename T>
auto sorted_insert(Cont& cont, T const& elem) {
  return cont.insert(find_item_location(cont, elem), elem);
}

template <search_dir Dir, typename Cont, typename TimeType>
auto get_pair_departing_after(Cont const& cont, TimeType limit) {
  return find_item_location(cont, limit, [](auto const& pair, auto const t) {
    if constexpr (Dir == search_dir::FWD) {
      return pair.first < t;
    } else {
      return pair.first > t;
    }
  });
}

// https://stackoverflow.com/questions/49318316/initialize-all-elements-or-stdarray-with-the-same-constructor-arguments
template <typename T, std::size_t N, std::size_t Idx = N>
struct array_maker {
  template <typename... Ts>
  static std::array<T, N> make_array(const T& v, Ts... tail) {
    return array_maker<T, N, Idx - 1>::make_array(v, v, tail...);
  }
};

template <typename T, std::size_t N>
struct array_maker<T, N, 1> {
  template <typename... Ts>
  static std::array<T, N> make_array(const T& v, Ts... tail) {
    return std::array<T, N>{v, tail...};
  }
};

}  // namespace csa
}  // namespace motis
