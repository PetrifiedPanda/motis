#pragma once

#include "utl/pipes.h"

#include "motis/csa/csa_search_shared.h"
#include "motis/csa/csa_timetable.h"

namespace motis::csa {

template <search_dir Dir, typename TargetStations, typename ArrivalTimes,
          typename TripReachable, typename FinalFootpaths>
struct csa_profile_reconstruction {
  using arrival_time_t = std::remove_reference_t<
      std::remove_cv_t<decltype(std::declval<FinalFootpaths>()[0])>>;
  using arr_dep_lst =
      std::remove_reference_t<decltype(std::declval<ArrivalTimes>()[0])>;

  static constexpr auto INVALID = INVALID_TIME<Dir, arrival_time_t>;

  csa_profile_reconstruction(csa_timetable const& tt,
                             std::map<station_id, time> const& start_times,
                             TargetStations const& target_stations,
                             ArrivalTimes const& arrival_time,
                             TripReachable const& trip_reachable,
                             FinalFootpaths const& final_footpaths)
      : tt_{tt},
        start_times_{start_times},
        target_stations_{target_stations},
        arrival_time_{arrival_time},
        trip_reachable_{trip_reachable},
        final_footpaths_{final_footpaths} {}

  bool is_target_station(station_id id) {
    return find_item_location(target_stations_, id) != target_stations_.end();
  }

  station_id find_meta_target(csa_station const* from) {
    auto const walk_duration = final_footpaths_[from->id_];

    auto const& footpaths =
        Dir == search_dir::FWD ? from->footpaths_ : from->incoming_footpaths_;
    for (auto const& fp : footpaths) {
      auto const to_station =
          Dir == search_dir::FWD ? fp.to_station_ : fp.from_station_;
      if (fp.duration_ == walk_duration && is_target_station(to_station)) {
        return to_station;
      }
    }

    return static_cast<station_id>(-1);
  }

  void add_final_con(csa_journey& j, csa_station const& from,
                     unsigned transfers) {
    assert(transfers == 0);
    if constexpr (Dir == search_dir::FWD) {
      auto const arr_time = j.arrival_time_;

      for (auto const& fp : from.footpaths_) {
        auto const& from_station = tt_.stations_[fp.to_station_];
        for (auto const* con : from_station.outgoing_connections_) {
          auto const& to_station = tt_.stations_[con->to_station_];
          if (con->arrival_ + fp.duration_ == arr_time &&
              is_target_station(to_station.id_)) {
            // TODO(root): not sure if enter should always be true here
            j.edges_.emplace_back(con->light_con_, &from_station, &to_station,
                                  true, true, con->departure_, arr_time);
            j.destination_station_ = &to_station;
            return;
          }
        }
      }
    } else {
      auto const dep_time = j.arrival_time_;
      for (auto const& fp : from.incoming_footpaths_) {
        auto const& to_station = tt_.stations_[fp.from_station_];
        for (auto const* con : to_station.incoming_connections_) {
          auto const& from_station = tt_.stations_[con->from_station_];
          if (con->departure_ - fp.duration_ == dep_time &&
              is_target_station(from_station.id_)) {
            // TODO(root): not sure if exit should always be true here
            j.edges_.emplace_back(con->light_con_, &from_station, &to_station,
                                  true, true, dep_time, con->arrival_);
            j.destination_station_ = &from_station;
            return;
          }
        }
      }
    }
    throw std::runtime_error{"Could not find suitable final connection"};
  }

  void extract_journey(csa_journey& j) {
    auto duration = Dir == search_dir::FWD ? j.arrival_time_ - j.start_time_
                                           : j.start_time_ - j.arrival_time_;
    if (duration == final_footpaths_[j.start_station_->id_]) {
      // Just walking from source stop is optimal
      auto const meta_target = find_meta_target(j.start_station_);
      assert(meta_target != static_cast<station_id>(-1));
      assert(j.transfers_ == 0);

      if constexpr (Dir == search_dir::FWD) {
        j.edges_.emplace_back(j.start_station_, &tt_.stations_[meta_target],
                              j.start_time_, j.arrival_time_);
      } else {
        j.edges_.emplace_back(&tt_.stations_[meta_target], j.start_station_,
                              j.arrival_time_, j.start_time_);
      }
    } else {
      constexpr search_dir RECON_DIR =
          Dir == search_dir::FWD ? search_dir::BWD : search_dir::FWD;
      auto const arrival = j.arrival_time_;

      auto departure = j.start_time_;
      auto* stop = j.start_station_;
      auto transfers = j.transfers_;
      for (; transfers > 0; --transfers) {
        auto [next_dep, jp] =
            get_journey_pointer(*stop, transfers, arrival, departure);

        if (jp.valid()) {
          auto const stop_id = Dir == search_dir::FWD
                                   ? jp.exit_con_->to_station_
                                   : jp.enter_con_->from_station_;
          stop = &tt_.stations_[stop_id];
          departure = next_dep;  // B: unsure about this
          add_journey_pointer_to_journey<RECON_DIR>(j, jp, tt_);
        } else {
          // TODO(root): This might actually have to do something other than
          // throwing an exception
          throw std::runtime_error{
              "csa_profile_reconstruction::get_journey_pointer returned "
              "invalid journey pointer"};
        }
      }

      assert(transfers == 0);
      // TODO(root): not sure if this if is necessary
      auto const& last_edge = j.edges_.back();
      auto const& last_station =
          Dir == search_dir::FWD ? *last_edge.to_ : *last_edge.from_;
      if (!is_target_station(last_station.id_)) {
        add_final_con(j, last_station, transfers);
      }

      if constexpr (RECON_DIR == search_dir::FWD) {
        std::reverse(begin(j.edges_), end(j.edges_));
      }
    }
  }

  auto get_fitting_arrival(arr_dep_lst const& arrival_time, time arrival,
                           time departure_limit, unsigned transfers) {
    // B: unsure about the "latest departure" part
    // find latest departure which arrives at the same time as the current
    // journey
    auto result = arrival_time.end();
    auto it = get_pair_departing_after<Dir>(arrival_time, departure_limit);
    for (; it != arrival_time.end(); ++it) {
      if (it->second[transfers] == arrival) {
        result = it;
      }
    }

    return result;
  }

  bool trip_arrives_early_enough(trip_id trip, unsigned transfers,
                                 time const arrival) const {
    if constexpr (Dir == search_dir::FWD) {
      return trip_reachable_[trip][transfers] <= arrival;
    } else {
      return trip_reachable_[trip][transfers] >= arrival;
    }
  }

  std::pair<arrival_time_t, journey_pointer> get_journey_pointer(
      csa_station const& station, unsigned transfers, time arrival,
      time departure) {
    if constexpr (Dir == search_dir::FWD) {
      for (auto const& fp : station.footpaths_) {
        auto const& enter_station = tt_.stations_[fp.to_station_];
        auto const new_departure = departure + fp.duration_;
        for (auto const* enter_con : get_enter_candidates(
                 enter_station, arrival, new_departure, transfers)) {
          // Go through trip_to con in reverse
          auto const& trip_to_con = tt_.trip_to_connections_[enter_con->trip_];
          for (auto it = std::rbegin(trip_to_con);; ++it) {
            auto const* exit_con = *it;

            auto const& arrival_time = arrival_time_[exit_con->to_station_];
            // B: unsure whether it should be transfers - 1 or just transfers
            if (auto exit_it = get_fitting_arrival(
                    arrival_time, arrival, exit_con->arrival_, transfers - 1);
                exit_it != arrival_time.end()) {
              return std::make_pair(exit_it->first,
                                    journey_pointer{enter_con, exit_con, &fp});
            } else if (exit_con == enter_con) {
              break;
            }
          }
        }
      }
    } else {
      // Swap departure and arrival because shadowing them causes a warning
      std::swap(departure, arrival);
      for (auto const& fp : station.incoming_footpaths_) {
        auto const& exit_station = tt_.stations_[fp.from_station_];
        auto const new_arr = arrival - fp.duration_;  // does this make sense?

        for (auto const* exit_con :
             get_exit_candidates(exit_station, new_arr, departure, transfers)) {
          auto const& trip_to_con = tt_.trip_to_connections_[exit_con->trip_];
          for (auto const& enter_con : trip_to_con) {
            auto const& arrival_time = arrival_time_[enter_con->from_station_];
            if (auto enter_it =
                    get_fitting_arrival(arrival_time, departure,
                                        enter_con->departure_, transfers - 1);
                enter_it != arrival_time.end()) {
              return std::make_pair(enter_it->first,
                                    journey_pointer{enter_con, exit_con, &fp});
            } else if (enter_con == exit_con) {
              break;
            }
          }
        }
      }
    }

    return std::make_pair(INVALID, journey_pointer{});
  }

  auto get_enter_candidates(csa_station const& departure_station, time arrival,
                            time departure, unsigned transfers) const {
    return utl::all(departure_station.outgoing_connections_) |
           utl::remove_if([this, arrival, departure,
                           transfers](csa_connection const* con) {
             return !trip_arrives_early_enough(con->trip_, transfers,
                                               arrival) ||
                    con->departure_ != departure || !con->from_in_allowed_;
           }) |
           utl::iterable();
  }

  auto get_exit_candidates(csa_station const& arrival_station, time arrival,
                           time departure, unsigned transfers) {
    return utl::all(arrival_station.incoming_connections_) |
           utl::remove_if([this, arrival, departure,
                           transfers](csa_connection const* con) {
             return !trip_arrives_early_enough(con->trip_, transfers,
                                               departure) ||
                    con->arrival_ != arrival || !con->to_out_allowed_;
           }) |
           utl::iterable();
  }

  csa_timetable const& tt_;
  std::map<station_id, time> const& start_times_;
  TargetStations const& target_stations_;

  ArrivalTimes const& arrival_time_;
  TripReachable const& trip_reachable_;
  FinalFootpaths const& final_footpaths_;
};

}  // namespace motis::csa