#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <list>
#include <vector>

#include "utl/verify.h"

#include "motis/core/schedule/interval.h"

#include "motis/csa/csa_journey.h"
#include "motis/csa/csa_profile_reconstruction.h"
#include "motis/csa/csa_search_shared.h"
#include "motis/csa/csa_statistics.h"
#include "motis/csa/csa_timetable.h"
#include "motis/csa/error.h"

namespace motis::csa::cpu {

template <search_dir Dir>
struct csa_profile_search {
  static constexpr auto INVALID = INVALID_TIME<Dir, time>;

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
        final_footpaths_(tt.stations_.size(), std::numeric_limits<time>::max()),
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
    sorted_insert(target_stations_, station.id_);

    set_final_footpaths(station);
  }

  void set_final_footpaths(csa_station const& target_station) {
    auto const& footpaths = Dir == search_dir::FWD
                                ? target_station.incoming_footpaths_
                                : target_station.footpaths_;
    for (auto const& fp : footpaths) {
      auto const station =
          Dir == search_dir::FWD ? fp.from_station_ : fp.to_station_;
      final_footpaths_[station] =
          std::min(fp.duration_, final_footpaths_[station]);
    }
  }

  static bool connection_is_later(csa_connection const& a,
                                  csa_connection const& b) {
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

  static time min(time t1, time t2) {
    if constexpr (Dir == search_dir::FWD) {
      return std::min(t1, t2);
    } else {
      return std::max(t1, t2);
    }
  }

  static arrival_times min(arrival_times const& arr1, arrival_times const& arr2,
                           arrival_times const& arr3) {
    auto result = arrival_times();
    for (auto i = 0; i < result.size(); ++i) {
      result[i] = min(arr1[i], min(arr2[i], arr3[i]));
    }

    return result;
  }

  static arrival_times min(arrival_times const& arr1,
                           arrival_times const& arr2) {
    auto result = arrival_times();
    for (auto i = 0; i < result.size(); ++i) {
      result[i] = min(arr1[i], arr2[i]);
    }

    return result;
  }

  static arrival_times shift(arrival_times const& arr) {
    auto result = array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID);
    for (auto i = 1; i < result.size(); ++i) {
      result[i] = arr[i - 1];
    }

    return result;
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

  arrival_times get_time_walking(station_id to_station, time arrival) {
    auto time_walking = INVALID;
    if (final_footpaths_[to_station] != std::numeric_limits<time>::max()) {
      if constexpr (Dir == search_dir::FWD) {
        time_walking = arrival + final_footpaths_[to_station];
      } else {
        time_walking = arrival - final_footpaths_[to_station];
      }
    }

    return array_maker<time, MAX_TRANSFERS + 1>::make_array(time_walking);
  }

  arrival_times get_time_transfer(station_id to_station, time arrival) {
    // find the first pair that departs after to_station arrival
    // B: is transfer_time_ the footpath to itself?
    auto const transfer_time = tt_.stations_[to_station].transfer_time_;
    auto const& list = arrival_time_[to_station];
    auto const limit = Dir == search_dir::FWD ? arrival + transfer_time
                                              : arrival - transfer_time;
    auto it = get_pair_departing_after<Dir>(list, limit);
    return shift(it->second);
  }

  static bool pair_comparator(std::pair<time, arrival_times> const& p1,
                              std::pair<time, arrival_times> const& p2) {
    if constexpr (Dir == search_dir::FWD) {
      return p1.first < p2.first;
    } else {
      return p1.first > p2.first;
    }
  }

  void add_arrival_time(std::pair<time, arrival_times> const& new_pair,
                        csa_station const& station) {
    auto& arrival_time = arrival_time_[station.id_];
    auto const insert_loc =
        find_item_location(arrival_time, new_pair, pair_comparator);
    auto const pair_to_insert = std::make_pair(
        new_pair.first, min(new_pair.second, insert_loc->second));
    arrival_time.insert(insert_loc, pair_to_insert);

    // B: If I understand this correctly, we don't need this
    // if we don't we can just emplace pair_to_insert into arrival_time
    for (auto it = std::prev(insert_loc); it != std::begin(arrival_time);
         --it) {
      if (dominates(pair_to_insert, *it)) {
        it = std::next(arrival_time.erase(it));  // This might be wrong
      }
    }
    if (dominates(pair_to_insert, *arrival_time.begin())) {
      arrival_time.erase(arrival_time.begin());
    }
  }

  void expand_footpaths(csa_station const& from_station, time station_arrival,
                        arrival_times const& best_arrival_times) {
    // B: the new pair might have to be the minimum of the first pair departing
    // after our arrival time and best_arrival_times
    auto const& footpaths = Dir == search_dir::FWD
                                ? from_station.incoming_footpaths_
                                : from_station.footpaths_;
    for (auto const& fp : footpaths) {
      auto const transfer_time = Dir == search_dir::FWD
                                     ? station_arrival - fp.duration_
                                     : station_arrival + fp.duration_;
      add_arrival_time(std::make_pair(transfer_time, best_arrival_times),
                       from_station);
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

      auto const to_station =
          Dir == search_dir::FWD ? con.to_station_ : con.from_station_;
      auto const from_station =
          Dir == search_dir::FWD ? con.from_station_ : con.to_station_;
      auto const arrival =
          Dir == search_dir::FWD ? con.arrival_ : con.departure_;
      auto const departure =
          Dir == search_dir::FWD ? con.departure_ : con.arrival_;

      auto const time_walking = get_time_walking(to_station, arrival);
      auto const time_trip = trip_reachable_[con.trip_];
      auto const time_transfer = get_time_transfer(to_station, arrival);

      auto const best_arrival_times =
          min(time_walking, time_trip, time_transfer);
      auto const best_pair = std::make_pair(departure, best_arrival_times);

      if (!is_dominated_in(best_pair, arrival_time_[to_station])) {
        expand_footpaths(tt_.stations_[from_station], departure,
                         best_arrival_times);
      }

      trip_reachable_[con.trip_] = best_arrival_times;
    }
  }

  std::vector<csa_journey> get_results(csa_station const& start_station,
                                       bool include_equivalent) {
    utl::verify_ex(!include_equivalent,
                   std::system_error{error::include_equivalent_not_supported});
    std::vector<csa_journey> journeys;
    auto recon = csa_profile_reconstruction<
        Dir, decltype(target_stations_), decltype(arrival_time_),
        decltype(trip_reachable_), decltype(final_footpaths_)>(
        tt_, start_times_, target_stations_, arrival_time_, trip_reachable_,
        final_footpaths_);
    for (auto const& pair : arrival_time_[start_station.id_]) {
      auto const departure = pair.first;
      auto const& station_arrival = pair.second;
      for (auto i = 0; i <= MAX_TRANSFERS; ++i) {
        auto const arrival_time = station_arrival[i];
        if (arrival_time != INVALID) {
          auto& journey =
              journeys.emplace_back(Dir, departure, arrival_time, i, nullptr);
          journey.start_station_ = &start_station;
          recon.extract_journey(journey);
        }
      }
    }

    return journeys;
  }

  csa_timetable const& tt_;
  interval search_interval_;

  std::map<station_id, time> start_times_;
  std::vector<station_id> target_stations_;

  // TODO(root): Time with different container types when the algorithm works
  // The pairs should be sorted by ascending departure_time (pair.first) when
  // inserted
  std::vector<std::list<std::pair<time, arrival_times>>> arrival_time_;

  std::vector<arrival_times> trip_reachable_;

  std::vector<time> final_footpaths_;

  csa_statistics& stats_;
};

}  // namespace motis::csa::cpu
