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

template <search_dir Dir, typename Cont, typename TimeType>
typename Cont::const_iterator get_pair_departing_after(Cont const& cont,
                                                       TimeType limit) {
  return std::lower_bound(cont.begin(), cont.end(), limit,
                          [](auto const& pair, auto const t) {
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
