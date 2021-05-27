#pragma once

#include <array>
#include <limits>
#include <list>
#include <vector>

#include "utl/verify.h"

#include "motis/core/schedule/interval.h"

#include "motis/csa/csa_journey.h"
#include "motis/csa/csa_search_shared.h"
#include "motis/csa/csa_statistics.h"
#include "motis/csa/csa_timetable.h"
#include "motis/csa/error.h"

namespace motis::csa::cpu {

template <search_dir Dir>
struct csa_profile_search {
  static constexpr auto INVALID = Dir == search_dir::FWD
                                      ? std::numeric_limits<time>::max()
                                      : std::numeric_limits<time>::min();

  using arrival_times = std::array<time, MAX_TRANSFERS + 1>;

  csa_profile_search(csa_timetable const& tt, interval const& search_interval,
                     csa_statistics& stats)
      : tt_{tt},
        search_interval_{search_interval},
        arrival_time_(
            tt.stations_.size(),
            {std::make_pair(
                INVALID,
                array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID))}),
        trip_reachable_(
            tt.trip_count_,
            array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID)),
        final_footpaths_(tt.stations_.size(), INVALID),
        stats_{stats} {}

  void add_start(csa_station const& station, time initial_duration) {
    // Ready for departure at station at time:
    // start time + initial_duration (Dir == search_dir::FWD)
    // start time - initial_duration (Dir == search_dir::BWD)
    stats_.start_count_++;
    auto const station_arrival =
        Dir == search_dir::FWD ? search_interval_.begin_ + initial_duration
                               : search_interval_.begin_ - initial_duration;

    start_times_[station.id_] = station_arrival;

    // TODO(root): Incorporate the station arrival into arrival_times_

    // TODO(root): call expand footpaths when implemented
  }

  void add_dest(csa_station const& station) {
    target_stations_.push_back(station.id_);

    set_final_footpaths(station);
  }

  void set_final_footpaths(csa_station const& target_station) {
    if constexpr (Dir == search_dir::FWD) {
      for (auto const& fp : target_station.incoming_footpaths_) {
        final_footpaths_[fp.from_station_] = fp.duration_;
      }
    } else {
      for (auto const& fp : target_station.footpaths_) {
        final_footpaths_[fp.to_station_] = fp.duration_;
      }
    }
  }

  static bool connection_is_earlier(csa_connection const& a,
                                    csa_connection const& b) {
    if constexpr (Dir == search_dir::FWD) {
      return a.departure_ < b.departure_;
    } else {
      return a.arrival_ > b.arrival_;
    }
  }

  // This should probably not be the exact complement
  static bool connection_is_later(csa_connection const& a,
                                  csa_connection const& b) {
    // return !connection_is_earlier(a, b);
    if constexpr (Dir == search_dir::FWD) {
      return a.departure_ > b.departure_;
    } else {
      return a.arrival_ < b.arrival_;
    }
  }

  /*
   * Maybe we should write a wrapper for std::array<time, MAX_TRANSFERS + 1>
   * that implements these operations
   */

  static arrival_times arr_min(
      arrival_times const& arr1, arrival_times const& arr2,
      arrival_times const& arr3 =
          array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID)) {
    auto result = arrival_times();
    for (auto i = 0; i < result.size(); ++i) {
      if constexpr (Dir == search_dir::FWD) {
        result[i] = std::min(std::min(arr1[i], arr2[i]), arr3[i]);
      } else {
        result[i] = std::max(std::max(arr1[i], arr2[i]), arr3[i]);
      }
    }

    return result;
  }

  static arrival_times arr_shift(arrival_times const& arr) {
    auto result = array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID);
    for (auto i = 1; i < result.size(); ++i) {
      result[i] = arr[i - 1];
    }

    return result;
  }

  static void arr_copy(arrival_times& to, arrival_times const& from) {
    for (auto i = 0; i < to.size(); ++i) {
      to[i] = from[i];
    }
  }

  static bool arr_equals(arrival_times const& arr1, arrival_times const& arr2) {
    for (auto i = 0; i < arr1.size(); ++i) {
      if (arr1[i] != arr2[i]) {
        return false;
      }
    }

    return true;
  }

  static bool dominates(std::pair<time, arrival_times> const& x,
                        std::pair<time, arrival_times> const& y) {
    if constexpr (Dir == search_dir::FWD) {
      if (y.first >= x.first) {
        // maybe this can be simplified to a range-based for loop
        for (auto i = 0; i < x.second.size(); ++i) {
          if (y.second[i] < x.second[i]) {
            return false;
          }
        }

        return true;
      }
    } else {
      if (y.first <= x.first) {
        // see comment above
        for (auto i = 0; i < x.second.size(); ++i) {
          if (y.second[i] > x.second[i]) {
            return false;
          }
        }

        return true;
      }
    }

    return false;
  }

  static bool is_dominated_in(
      std::pair<time, arrival_times> const& pair,
      std::list<std::pair<time, arrival_times>> const& list) {
    return std::any_of(list.begin(), list.end(), [&](auto const& other) {
      return dominates(other, pair);
    });
  }

  arrival_times get_time_walking(csa_connection const& con) {
    auto time_walking = INVALID;
    if constexpr (Dir == search_dir::FWD) {
      if (final_footpaths_[con.to_station_] != INVALID) {
        time_walking = con.arrival_ + final_footpaths_[con.to_station_];
      }
    } else {
      if (final_footpaths_[con.from_station_] != INVALID) {
        time_walking = con.departure_ - final_footpaths_[con.from_station_];
      }
    }

    return array_maker<time, MAX_TRANSFERS + 1>::make_array(time_walking);
  }

  const arrival_times& get_time_transfer(csa_connection const& con) {
    // find the first pair that departs after station arrival
    auto const station_id =
        Dir == search_dir::FWD ? con.to_station_ : con.from_station_;
    // B: is transfer_time_ the footpath to itself?
    auto const transfer_time = tt_.stations_[station_id].transfer_time_;
    auto const& list = arrival_time_[station_id];
    auto const limit = Dir == search_dir::FWD ? con.arrival_ + transfer_time
                                              : con.departure_ - transfer_time;
    auto it = std::lower_bound(list.begin(), list.end(), limit,
                               [](auto const& p, auto const& t) {
                                 if constexpr (Dir == search_dir::FWD) {
                                   return p.first < t;
                                 } else {
                                   return p.first > t;
                                 }
                               });
    return it->second;
  }

  void expand_footpaths(csa_station const& station, time station_arrival,
                        arrival_times const& best_arrival_times) {
    if constexpr (Dir == search_dir::FWD) {
      for (auto const& fp : station.incoming_footpaths_) {
        auto const new_pair =
            std::make_pair(station_arrival - fp.duration_, best_arrival_times);
        (void)new_pair;
        // TODO(root) insert into arrival_time_[station.id_]
      }
    } else {
      for (auto const& fp : station.footpaths_) {
        auto const new_pair =
            std::make_pair(station_arrival + fp.duration_, best_arrival_times);
        (void)new_pair;
        // TODO(root) insert into arrival_time_[station.id_]
      }
    }
  }

  void search() {
    auto const& connections =
        Dir == search_dir::FWD ? tt_.fwd_connections_ : tt_.bwd_connections_;

    // Assume the connections are sorted by increasing c.departure_time
    csa_connection const earliest_possible_con{search_interval_.begin_};
    csa_connection const latest_possible_con{search_interval_.end_};

    auto const range_start =
        std::lower_bound(rbegin(connections), rend(connections),
                         latest_possible_con, connection_is_later);

    auto const range_end =
        std::upper_bound(rbegin(connections), rend(connections),
                         earliest_possible_con, connection_is_later);

    // Run algorithm
    for (auto it = range_start; it != range_end; ++it) {
      auto const& con = *it;

      auto const time_walking = get_time_walking(con);
      auto const time_trip = trip_reachable_[con.trip_];
      auto const time_transfer = arr_shift(get_time_transfer(con));

      auto const best_arrival_times =
          arr_min(time_walking, time_trip, time_transfer);
      auto const departure_time =
          Dir == search_dir::FWD ? con.departure_ : con.arrival_;
      auto const best_pair = std::make_pair(departure_time, best_arrival_times);

      auto const to_index =
          Dir == search_dir::FWD ? con.from_station_ : con.to_station_;

      if (!is_dominated_in(best_pair, arrival_time_[to_index])) {
        auto const& station = Dir == search_dir::FWD
                                  ? tt_.stations_[con.from_station_]
                                  : tt_.stations_[con.to_station_];
        auto const station_arrival =
            Dir == search_dir::FWD ? con.departure_ : con.arrival_;
        expand_footpaths(station, station_arrival, best_arrival_times);
      }

      arr_copy(trip_reachable_[con.trip_], best_arrival_times);
    }
  }

  std::vector<csa_journey> get_results(csa_station const& station,
                                       bool include_equivalent) {
    utl::verify_ex(!include_equivalent,
                   std::system_error{error::include_equivalent_not_supported});

    (void)station;
    return {};
  }

  csa_timetable const& tt_;
  interval search_interval_;

  std::map<station_id, time> start_times_;
  std::vector<station_id> target_stations_;  // B: not sure if we will need this

  // TODO(root): Time with different container types when the algorithm works
  // The pairs should be sorted by ascending departure_time (pair.first) when
  // inserted
  std::vector<std::list<std::pair<time, arrival_times>>> arrival_time_;

  std::vector<arrival_times> trip_reachable_;

  std::vector<time> final_footpaths_;

  csa_statistics& stats_;
};

}  // namespace motis::csa::cpu
