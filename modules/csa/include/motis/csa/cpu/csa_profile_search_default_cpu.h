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
        stats_{stats} {
    std::cout << "From: " << search_interval.begin_
              << " To: " << search_interval.end_ << std::endl;
  }

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
                           search_interval_.begin_)));
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

  static bool is_dominated_in(
      std::pair<time, arrival_times> const& pair,
      std::list<std::pair<time, arrival_times>> const& list) {
    return std::any_of(list.begin(), list.end(), [&](auto const& other) {
      return other.second == pair.second ||
             (time_comp(pair.first, other.first) &&
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

  static void print_dep_arr(std::pair<time, arrival_times> const& p) {
    std::cout << "Departure: " << p.first << std::endl;
    std::cout << "Arrivals: ";
    for (auto const& t : p.second) {
      std::cout << t << ", ";
    }
    std::cout << std::endl;
  }

  static std::string station_str(csa_station const& s) {
    if (s.station_ptr_ == nullptr) return "";
    return s.station_ptr_->name_.str();
  }

  void add_arrival_time(std::pair<time, arrival_times> const& new_pair,
                        csa_station const& station) {
    auto& arrival_time = arrival_time_[station.id_];
    auto const insert_loc =
        find_item_location(arrival_time, new_pair, pair_comparator);
    auto const inserted = arrival_time.insert(
        insert_loc, std::make_pair(new_pair.first,
                                   min(new_pair.second, insert_loc->second)));

    // Delete all dominated or same pairs departing earlier than current
    for (auto it = std::make_reverse_iterator(inserted);
         it != std::rend(arrival_time);) {
      if (inserted->second == it->second ||
          dominates(inserted->second, it->second)) {
        // This might be wrong
        /*
        std::cout << "Deleted earlier from profile of " << station_str(station)
                  << " : " << std::endl;
        print_dep_arr(*it);
        std::cout << "For pair: " << std::endl;
        print_dep_arr(*inserted);
        std::cout << "\n\n" << std::endl;
         */
        utl::verify_ex(
            it->second == std::next(it).base()->second &&
                it->first == std::next(it).base()->first,
            std::runtime_error{"Not deleting the thing we want to delete"});
        it = std::make_reverse_iterator(
            arrival_time.erase(std::next(it).base()));
      } else {
        ++it;
      }
    }

    // Delete all dominated or same pairs departing at the same time as current
    for (auto it = std::next(inserted); it != std::end(arrival_time);) {
      if (inserted->first != it->first) {
        break;
      } else if (inserted->second == it->second ||
                 dominates(inserted->second, it->second)) {
        /*
        std::cout << "Deleted same time from profile of "
                  << station_str(station) << " : " << std::endl;
        print_dep_arr(*it);
        std::cout << "For pair: " << std::endl;
        print_dep_arr(*inserted);
        std::cout << "\n\n" << std::endl;
         */
        it = arrival_time.erase(it);
      } else {
        ++it;
      }
    }
  }

  void expand_footpaths(csa_station const& from_station, time station_arrival,
                        arrival_times const& best_arrival_times) {
    auto const& footpaths = Dir == search_dir::FWD
                                ? from_station.incoming_footpaths_
                                : from_station.footpaths_;
    for (auto const& fp : footpaths) {
      auto const& fp_from =
          Dir == search_dir::FWD ? fp.from_station_ : fp.to_station_;
      auto const transfer_time = Dir == search_dir::FWD
                                     ? station_arrival - fp.duration_
                                     : station_arrival + fp.duration_;
      add_arrival_time(std::make_pair(transfer_time, best_arrival_times),
                       tt_.stations_[fp_from]);
    }
  }

  void search() {
    auto const& connections =
        Dir == search_dir::FWD ? tt_.fwd_connections_ : tt_.bwd_connections_;

    // Assume the connections are sorted by increasing c.departure_time
    csa_connection const earliest_possible_con{search_interval_.begin_};

    auto const range_start = std::rbegin(connections);

    auto const range_end =
        std::upper_bound(rbegin(connections), rend(connections),
                         earliest_possible_con, connection_is_later);

    auto const invalid_arrivals =
        array_maker<time, MAX_TRANSFERS + 1>::make_array(INVALID);
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

      bool const out_allowed = Dir == search_dir::FWD ? con.to_out_allowed_ : con.from_in_allowed_;
      bool const in_allowed = Dir == search_dir::FWD ? con.from_in_allowed_ : con.to_out_allowed_;

      // if con.to_out_allowed_
      auto const time_walking = get_time_walking(to_station, arrival);
      auto const& time_trip = trip_reachable_[con.trip_];
      // if con.to_out_allowed_
      auto const time_transfer = get_time_transfer(to_station, arrival);

      auto const best_pair = std::make_pair(
          departure, min(time_walking, time_trip, time_transfer));
      auto const& best_arrival_times = best_pair.second;
      // if con.from_in_allowed_
      if (best_pair.second != invalid_arrivals &&
          !is_dominated_in(best_pair, arrival_time_[from_station])) {
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

    if (arrival_time_[start_station.id_].size() != 1) {
      std::cout << station_str(start_station) << std::endl;
      for (auto const& p : arrival_time_[start_station.id_]) {
        if (p.first != INVALID) {
          print_dep_arr(p);
          std::cout << std::endl;
        }
      }
      std::cout << "\n\n" << std::endl;
    }


    for (auto const& s : tt_.stations_) {
      if (station_str(s) == "Kalkriese Kreuzung Ellerholz, Bramsche") {
        for (auto const& p : arrival_time_[s.id_]) {
          if (p.first != INVALID) {
            print_dep_arr(p);
            std::cout << std::endl;
          }
        }
      }
    }

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
      if (time_comp(dep, search_interval_.end_) || dep == search_interval_.end_) {
        for (auto i = 0; i < arrs.size(); ++i) {
          auto const arr = arrs[i];
          if (time_comp(arr, min_reconstructed[i])) {
            auto& journey = journeys.emplace_back(Dir, dep, arr, i, nullptr);
            journey.start_station_ = &start_station;
            std::cout << "Got journey with " << i << " transfers" << std::endl;
            recon.extract_journey(journey);
            std::cout << "Got journey " << std::endl;
            min_reconstructed[i] = arr;
            for (auto j = i + 1; j < arrs.size(); ++j) {
              if (time_comp(arr, min_reconstructed[j])) {
                min_reconstructed[j] = arr;
              }
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
