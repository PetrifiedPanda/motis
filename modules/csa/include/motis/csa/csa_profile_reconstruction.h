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

  void add_final_footpath(csa_journey& j, csa_station const& from,
                          time arrival) {
    auto const walk_duration = final_footpaths_[from.id_];

    auto const& footpaths =
        Dir == search_dir::FWD ? from.footpaths_ : from.incoming_footpaths_;
    for (auto const& fp : footpaths) {
      auto const& to_station =
          Dir == search_dir::FWD ? fp.to_station_ : fp.from_station_;
      if (fp.duration_ == walk_duration && is_target_station(to_station)) {
        j.destination_station_ = &tt_.stations_[to_station];
        if constexpr (Dir == search_dir::FWD) {
          j.edges_.emplace_back(&from, j.destination_station_, arrival,
                                arrival + fp.duration_);
        } else {
          j.edges_.emplace_back(j.destination_station_, &from,
                                arrival - fp.duration_, arrival);
        }
        return;
      }
    }

    throw std::runtime_error{"Could not find fitting final footpath"};
  }

  void add_final_con(csa_journey& j, csa_station const& from, time arrival,
                     unsigned transfers) {
    assert(transfers == 0);
    if constexpr (Dir == search_dir::FWD) {
      auto const arr_time = j.arrival_time_;
      for (auto const& enter_fp : from.footpaths_) {
        auto const& from_station = tt_.stations_[enter_fp.to_station_];
        for (auto const* con : from_station.outgoing_connections_) {
          if (arrival + enter_fp.duration_ <= con->departure_) {
            auto const& to_station = tt_.stations_[con->to_station_];
            for (auto const& exit_fp : to_station.footpaths_) {
              if (is_target_station(exit_fp.to_station_) &&
                  con->arrival_ + exit_fp.duration_ == arr_time) {
                if (enter_fp.from_station_ != enter_fp.to_station_) {
                  j.edges_.emplace_back(&from, &from_station, arrival,
                                        arrival + enter_fp.duration_);
                }
                j.edges_.emplace_back(con->light_con_, &from_station,
                                      &to_station, true, true, con->departure_,
                                      con->arrival_);
                if (exit_fp.from_station_ != exit_fp.to_station_) {
                  j.edges_.emplace_back(&to_station,
                                        &tt_.stations_[exit_fp.to_station_],
                                        con->arrival_, arr_time);
                }
                j.destination_station_ = &tt_.stations_[exit_fp.to_station_];
                return;
              }
            }
          }
        }
      }
    } else {
      auto const dep_time = j.arrival_time_;
      for (auto const& exit_fp : from.incoming_footpaths_) {
        auto const& to_station = tt_.stations_[exit_fp.from_station_];
        for (auto const* con : to_station.incoming_connections_) {
          if (arrival - exit_fp.duration_ >= con->arrival_) {
            auto const& from_station = tt_.stations_[con->from_station_];
            for (auto const& enter_fp : from_station.incoming_footpaths_) {
              if (is_target_station(enter_fp.from_station_) &&
                  con->departure_ - enter_fp.duration_ == dep_time) {
                if (exit_fp.from_station_ != exit_fp.to_station_) {
                  j.edges_.emplace_back(&to_station, &from, con->arrival_,
                                        arrival);  // TODO(root): unsure
                }
                j.edges_.emplace_back(con->light_con_, &from_station,
                                      &to_station, true, true, con->departure_,
                                      con->arrival_);
                if (enter_fp.from_station_ != enter_fp.to_station_) {
                  j.edges_.emplace_back(
                      &tt_.stations_[enter_fp.from_station_], &from_station,
                      dep_time,
                      dep_time + enter_fp.duration_);  // TODO(root): unsure
                }
                j.destination_station_ = &tt_.stations_[enter_fp.from_station_];
                return;
              }
            }
          }
        }
      }
    }

    throw std::runtime_error{"Could not find suitable final connection"};
  }

  static std::string station_str(csa_station const& s) {
    if (s.station_ptr_ == nullptr) return "";
    return s.station_ptr_->name_.str();
  }

  void extract_journey(csa_journey& j) {
    std::cout << "Extracting journey" << std::endl;
    auto duration = Dir == search_dir::FWD ? j.arrival_time_ - j.start_time_
                                           : j.start_time_ - j.arrival_time_;
    if (duration == final_footpaths_[j.start_station_->id_]) {
      // Just walking from source stop is optimal
      // TODO(root): use add_final_footpath here
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
      int transfers = j.transfers_;
      for (; transfers >= 0; --transfers) {
        auto [next_dep, jp] =
            get_journey_pointer(*stop, transfers, arrival, departure);
        std::cout << "Got journey pointer" << std::endl;
        std::cout << "From: " << jp.enter_con_->departure_ << " "
                  << station_str(tt_.stations_[jp.enter_con_->from_station_])
                  << std::endl;
        std::cout << "To: " << jp.exit_con_->arrival_ << " "
                  << station_str(tt_.stations_[jp.exit_con_->to_station_])
                  << std::endl;
        std::cout << std::endl;

        assert(jp.valid());
        auto const stop_id = Dir == search_dir::FWD
                                 ? jp.exit_con_->to_station_
                                 : jp.enter_con_->from_station_;
        stop = &tt_.stations_[stop_id];
        departure = next_dep;  // B: unsure about this
        add_journey_pointer_to_journey<RECON_DIR>(j, jp, tt_);
      }
      assert(!j.edges_.empty());
      auto const& last_edge = j.edges_.back();
      auto const& last_station =
          Dir == search_dir::FWD ? *last_edge.to_ : *last_edge.from_;
      if (!is_target_station(last_station.id_)) {
        auto const arr =
            Dir == search_dir::FWD ? last_edge.arrival_ : last_edge.departure_;
        add_final_footpath(j, last_station, arr);
      } else {
        j.destination_station_ = &last_station;
      }

      if constexpr (RECON_DIR == search_dir::FWD) {
        std::reverse(begin(j.edges_), end(j.edges_));
      }
    }
  }

  void extract_journey_include_equivalent(std::vector<csa_journey>& journeys) {
    // TODO(root):
    throw std::runtime_error{"Include Equivalent not implemented"};
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
      return trip_reachable_[trip][transfers] == arrival;
    } else {
      return trip_reachable_[trip][transfers] == arrival;
    }
  }

  std::pair<arrival_time_t, journey_pointer> get_journey_pointer(
      csa_station const& station, unsigned transfers, time arrival,
      time departure) {
    auto const second_transfers = transfers == 0 ? 0 : transfers - 1;
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
            if (auto exit_it =
                    get_fitting_arrival(arrival_time, arrival,
                                        exit_con->arrival_, second_transfers);
                exit_it != arrival_time.end()) {
              return std::make_pair(exit_it->first,
                                    journey_pointer{enter_con, exit_con, &fp});
            } else if (exit_con == enter_con) {
              return std::make_pair(departure,
                                    journey_pointer{enter_con, exit_con, &fp});
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
            if (auto enter_it = get_fitting_arrival(arrival_time, departure,
                                                    enter_con->departure_,
                                                    second_transfers);
                enter_it != arrival_time.end()) {
              return std::make_pair(enter_it->first,
                                    journey_pointer{enter_con, exit_con, &fp});
            } else if (enter_con == exit_con) {
              return std::make_pair(departure,
                                    journey_pointer{enter_con, exit_con, &fp});
            }
          }
        }
      }
    }

    throw std::runtime_error{"Could not find fitting journey pointer"};
  }

  auto get_enter_candidates(csa_station const& departure_station, time arrival,
                            time departure, unsigned transfers) const {
    return utl::all(departure_station.outgoing_connections_) |
           utl::remove_if([this, arrival, departure,
                           transfers](csa_connection const* con) {
             return !trip_arrives_early_enough(con->trip_, transfers,
                                               arrival) ||
                    con->departure_ !=
                        departure /* || !con->from_in_allowed_ */;
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