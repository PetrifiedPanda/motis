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

// TODO(root): from_in_allowed_ and to_out_allowed_ are ignored
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

    // TODO(root): Not sure if we need to do more here
  }

  void add_dest(csa_station const& station) {
    stats_.destination_count_++;
    sorted_insert(target_stations_, station.id_);

    set_final_footpaths(station);
    arrival_time_[station.id_].push_front(
        std::make_pair(search_interval_.begin_,
                       array_maker<time, MAX_TRANSFERS + 1>::make_array(
                           search_interval_.end_)));
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

  inline static bool time_comp(time t1, time t2) {
    if constexpr (Dir == search_dir::FWD) {
      return t1 < t2;
    } else {
      return t1 > t2;
    }
  }

  // TODO(root): Remove useless code duplication for dominates methods
  static bool dominates(arrival_times const& x, arrival_times const& y) {
    bool x_has_better = false;
    for (auto i = 0; i < x.size(); ++i) {
      if (time_comp(y[i], x[i])) {
        return false;
      } else if (time_comp(x[i], y[i])) {
        x_has_better = true;
      }
    }

    return x_has_better;
  }

  static bool dominates(std::pair<time, arrival_times> const& x,
                        std::pair<time, arrival_times> const& y) {
    bool x_has_better = time_comp(x.first, y.first);

    if (!time_comp(y.first, x.first)) {
      for (auto i = 0; i < x.second.size(); ++i) {
        if (time_comp(x.second[i], y.second[i])) {
          x_has_better = true;
        } else if (time_comp(y.second[i], x.second[i])) {
          return false;
        }
      }

      return x_has_better;
    } else {
      return false;
    }
  }

  static bool is_dominated_in(
      std::pair<time, arrival_times> const& pair,
      std::list<std::pair<time, arrival_times>> const& list) {
    return std::any_of(list.begin(), list.end(), [&](auto const& other) {
      return dominates(other, pair) || (time_comp(pair.first, other.first) &&
                                        dominates(other.second, pair.second));
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
    if (it == list.end()) {
      return array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID);
    } else {
      return shift(it->second);
    }
  }

  inline static bool pair_comparator(std::pair<time, arrival_times> const& p1,
                                     std::pair<time, arrival_times> const& p2) {
    return time_comp(p1.first, p2.first);
  }

  void add_arrival_time(std::pair<time, arrival_times> const& new_pair,
                        csa_station const& station) {
    auto& arrival_time = arrival_time_[station.id_];
    auto const insert_loc =
        find_item_location(arrival_time, new_pair, pair_comparator);
    auto const inserted = arrival_time.insert(
        insert_loc, std::make_pair(new_pair.first,
                                   min(new_pair.second, insert_loc->second)));

    if (inserted != arrival_time.begin()) {
      using reverse =
          std::list<std::pair<time, arrival_times>>::const_reverse_iterator;
      // Delete all dominated pairs departing earlier than current
      for (auto it = reverse(std::prev(inserted));
           it != std::rend(arrival_time); ++it) {
        if (dominates(inserted->second, it->second)) {
          // This might be wrong
          it = std::prev(reverse(arrival_time.erase(it.base())));
        }
      }
    }

    // Delete all dominated pairs departing at the same time as current
    for (auto it = insert_loc; it != std::end(arrival_time); ++it) {
      if (it->first != inserted->first) {
        break;
      } else if (dominates(inserted->second, it->second)) {
        it = std::prev(arrival_time.erase(it));
      }
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
      auto const& time_trip = trip_reachable_[con.trip_];
      auto const time_transfer = get_time_transfer(to_station, arrival);

      auto const best_pair = std::make_pair(
          departure, min(time_walking, time_trip, time_transfer));
      auto const& best_arrival_times = best_pair.second;

      if (!is_dominated_in(best_pair, arrival_time_[from_station])) {
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

    /*
     * Go through profiles in reverse order, always saving the earliest arrival
     * with each number of transfers. Because the latest arrivals are in the
     * latest profiles, we only need to check if the current arrival is smaller
     * than the last extracted arrival with this amount of transfers
     */
    auto const& arr_time = arrival_time_[start_station.id_];
    auto min_reconstructed =
        array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID);
    for (auto it = std::next(std::rbegin(arr_time)); it != std::rend(arr_time);
         ++it) {
      auto const& [dep, arrs] = *it;
      for (auto i = 0; i < arrs.size(); ++i) {
        auto const arr = arrs[i];
        if (time_comp(arr, min_reconstructed[i])) {
          auto& journey = journeys.emplace_back(Dir, dep, arr, i, nullptr);
          journey.start_station_ = &start_station;
          recon.extract_journey(journey);
          min_reconstructed[i] = arr;
          for (auto j = i + 1; j < arrs.size(); ++j) {
            if (time_comp(arr, min_reconstructed[j])) {
              min_reconstructed[j] = arr;
            }
          }
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
