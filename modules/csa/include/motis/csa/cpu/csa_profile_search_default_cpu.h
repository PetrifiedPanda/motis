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

  template <typename T>
  using vec_of_vec = std::vector<std::vector<T>>;
  using arrival_times = std::array<time, MAX_TRANSFERS + 1>;

  csa_profile_search(csa_timetable const& tt, interval const& search_interval,
                     csa_statistics& stats)
      : tt_{tt},
        search_interval_{search_interval},
        arrival_time_(
            tt.stations_.size(),
            std::vector<std::list<std::pair<time, arrival_times>>>(
                tt.stations_.size(),
                {std::make_pair(
                    INVALID, array_maker<time, MAX_TRANSFERS + 1>::make_array(
                                 INVALID))})),
        trip_reachable_(
            tt.stations_.size(),
            std::vector<arrival_times>(
                tt.trip_count_,
                array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID))),
        final_footpaths_(tt.stations_.size(),
                         std::vector<time>(tt.stations_.size(), INVALID)),
        stats_{stats} {}

  void add_start(csa_station const& station, time initial_duration) {
    // Ready for departure at station at time:
    // start time + initial_duration (Dir == search_dir::FWD)
    // start time - initial_duration (Dir == search_dir::BWD)
    stats_.start_count_++;

    (void)station;
    (void)initial_duration;
  }

  /*
   * Helper that sets final_footpaths_ according to setter_func
   * to avoid duplicating code for set_final_footpaths and reset_final_footpaths
   */
  template <typename Func>
  void set_final_footpaths(Func setter_func) {
    for (auto const& target_station : tt_.stations_) {
      auto const target_id = target_station.id_;

      if constexpr (Dir == search_dir::FWD) {
        for (auto const& fp : target_station.incoming_footpaths_) {
          final_footpaths_[target_id][fp.from_station_] = setter_func(fp);
        }
      } else {
        for (auto const& fp : target_station.footpaths_) {
          final_footpaths_[target_id][fp.to_station_] = setter_func(fp);
        }
      }
    }
  }

  void set_final_footpaths() {
    set_final_footpaths([](footpath const& fp) { return fp.duration_; });
  }

  void reset_final_footpaths() {
    set_final_footpaths([](footpath const& fp) {
      (void)fp;
      return INVALID;
    });
  }

  static bool connection_comparator(csa_connection const& a,
                                    csa_connection const& b) {
    if constexpr (Dir == search_dir::FWD) {
      return a.departure_ < b.departure_;
    } else {
      return a.arrival_ > b.arrival_;
    }
  }

  static bool connection_comparator_complement(csa_connection const& a,
                                               csa_connection const& b) {
    return !connection_comparator(a, b);
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
        result[i] = std::max(std::max(arr1[i], arr2[i]),
                             arr3[i]);  // B: not sure if this is correct
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

  static arrival_times get_time_walking(
      csa_connection const& con, std::vector<time> const& final_footpaths) {
    auto time_walking = INVALID;
    if constexpr (Dir == search_dir::FWD) {
      if (final_footpaths[con.to_station_] != INVALID) {
        time_walking = con.arrival_ + final_footpaths[con.to_station_];
      }
    } else {
      if (final_footpaths[con.from_station_] != INVALID) {
        time_walking = con.departure_ - final_footpaths[con.from_station_];
      }
    }

    return array_maker<time, MAX_TRANSFERS + 1>::make_array(time_walking);
  }

  arrival_times get_time_transfer(
      csa_connection const& con,
      std::vector<std::list<std::pair<time, arrival_times>>> const&
          arrival_time) {
    // TODO(root)
    // what do they mean by evaluate S at c_arr_time

    (void)con;
    (void)arrival_time;
    return array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID);
  }

  void search_with_target_station(
      csa_station const& target_station,
      std::vector<csa_connection>::const_reverse_iterator const& range_start,
      std::vector<csa_connection>::const_reverse_iterator const& range_end) {
    auto const target_id = target_station.id_;
    auto& arrival_time = arrival_time_[target_id];
    auto& trip_reachable = trip_reachable_[target_id];
    auto const& final_footpaths = final_footpaths_[target_id];

    for (auto it = range_start; it != range_end; ++it) {
      auto const& con = *it;

      auto const time_walking = get_time_walking(con, final_footpaths);
      auto const time_trip = trip_reachable[con.trip_];
      auto const time_transfer = get_time_transfer(con, arrival_time);

      auto const best_time = arr_min(time_walking, time_trip, time_transfer);

      auto const y = Dir == search_dir::FWD  // TODO(root): rename
                         ? arrival_time[con.from_station_].front().second
                         : arrival_time[con.to_station_].front().second;

      auto best_of_both = arr_min(best_time, y);

      // This is probably not correct
      if (!arr_equals(y, best_of_both)) {
        auto const& footpaths =
            Dir == search_dir::FWD
                ? tt_.stations_[con.to_station_].incoming_footpaths_
                : tt_.stations_[con.from_station_].footpaths_;
        for (auto const& fp : footpaths) {
          // what does "incorporate into profile" mean?
          if constexpr (Dir == search_dir::FWD) {
            auto const arrival = con.departure_ - fp.duration_;
            arrival_time[con.from_station_].push_front(
                std::make_pair(arrival, best_of_both));
          } else {
            auto const arrival = con.arrival_ + fp.duration_;
            arrival_time[con.to_station_].push_front(
                std::make_pair(arrival, best_of_both));
          }
        }
      }

      arr_copy(trip_reachable[con.trip_], best_time);
    }
  }

  void search() {
    auto const& connections =
        Dir == search_dir::FWD ? tt_.fwd_connections_ : tt_.bwd_connections_;
    set_final_footpaths();

    // Assume the connections are sorted by decreasing c.departure_time
    csa_connection const earliest_possible_con{search_interval_.begin_};
    csa_connection const latest_possible_con{search_interval_.end_};

    auto const range_start =
        std::lower_bound(rbegin(connections), rend(connections),
                         latest_possible_con, connection_comparator_complement);

    auto const range_end = std::upper_bound(
        rbegin(connections), rend(connections), earliest_possible_con,
        connection_comparator_complement);

    // Run the algorithm with each station as target station (probably too slow)
    for (auto const& target_station : tt_.stations_) {
      search_with_target_station(target_station, range_start, range_end);
    }

    // B: this may not be necessary if instances are only used once
    reset_final_footpaths();
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

  // TODO(root): Time with different container types when the algorithm works
  // The pairs should be sorted by departure time in the algorithm,
  // because the connections are traversed by decreasing departure time
  vec_of_vec<std::list<std::pair<time, arrival_times>>> arrival_time_;

  vec_of_vec<arrival_times> trip_reachable_;

  vec_of_vec<time> final_footpaths_;

  csa_statistics& stats_;
};

}  // namespace motis::csa::cpu
