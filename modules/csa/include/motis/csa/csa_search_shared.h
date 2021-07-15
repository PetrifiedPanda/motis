#pragma once

#include <algorithm>
#include <array>
#include <limits>

#include "motis/core/schedule/time.h"
#include "motis/csa/csa_journey.h"
#include "motis/csa/csa_timetable.h"

namespace motis {
namespace csa {

constexpr duration MAX_TRAVEL_TIME = 1440;
constexpr duration MAX_TRANSFERS = 7;

template <search_dir Dir, typename TimeType>
constexpr auto INVALID_TIME = Dir == search_dir::FWD
                                  ? std::numeric_limits<TimeType>::max()
                                  : std::numeric_limits<TimeType>::min();

struct journey_pointer {
  journey_pointer() = default;
  journey_pointer(csa_connection const* enter_con,
                  csa_connection const* exit_con, footpath const* footpath)
      : enter_con_(enter_con), exit_con_(exit_con), footpath_(footpath) {}

  bool valid() const {
    return enter_con_ != nullptr && exit_con_ != nullptr &&
           footpath_ != nullptr;
  }

  csa_connection const* enter_con_{nullptr};
  csa_connection const* exit_con_{nullptr};
  footpath const* footpath_{nullptr};
};

template <search_dir ReconDir>
void add_journey_pointer_to_journey(csa_journey& j, journey_pointer const& jp,
                                    csa_timetable const& tt) {
  if (jp.footpath_->from_station_ != jp.footpath_->to_station_) {
    if constexpr (ReconDir == search_dir::FWD) {
      j.edges_.emplace_back(
          &tt.stations_[jp.footpath_->from_station_],
          &tt.stations_[jp.footpath_->to_station_], jp.exit_con_->arrival_,
          jp.exit_con_->arrival_ + jp.footpath_->duration_, -1);
    } else {
      j.edges_.emplace_back(&tt.stations_[jp.footpath_->from_station_],
                            &tt.stations_[jp.footpath_->to_station_],
                            jp.enter_con_->departure_ - jp.footpath_->duration_,
                            jp.enter_con_->departure_, -1);
    }
  }
  assert(jp.enter_con_->trip_ == jp.exit_con_->trip_);
  auto const& trip_cons = tt.trip_to_connections_[jp.exit_con_->trip_];
  auto const add_trip_edge = [&](csa_connection const* con) {
    auto const enter = con == jp.enter_con_;
    auto const exit = con == jp.exit_con_;
    utl::verify(con->light_con_ != nullptr, "invalid light connection");
    j.edges_.emplace_back(con->light_con_, &tt.stations_[con->from_station_],
                          &tt.stations_[con->to_station_], enter, exit,
                          con->departure_, con->arrival_);
  };
  if constexpr (ReconDir == search_dir::FWD) {
    auto in_trip = false;
    for (auto it = std::rbegin(trip_cons); it != std::rend(trip_cons); ++it) {
      auto const* con = *it;
      if (con == jp.exit_con_) {
        in_trip = true;
      }
      if (in_trip) {
        add_trip_edge(con);
      }
      if (con == jp.enter_con_) {
        break;
      }
    }
  } else {
    auto in_trip = false;
    for (auto const& con : trip_cons) {
      if (con == jp.enter_con_) {
        in_trip = true;
      }
      if (in_trip) {
        add_trip_edge(con);
      }
      if (con == jp.exit_con_) {
        break;
      }
    }
  }
}

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
