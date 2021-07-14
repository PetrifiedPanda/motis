#pragma once

#include "motis/csa/csa_reconstruction.h"

namespace motis::csa {

template <search_dir Dir, typename TargetStations, typename ArrivalTimes,
          typename TripReachable, typename FinalFootpaths>
struct csa_profile_reconstruction {
  using arrival_time_t = std::remove_reference_t<
      std::remove_cv_t<decltype(std::declval<FinalFootpaths>()[0])>>;

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
    // TODO(root): BIDIR
    auto walk_duration = final_footpaths_[from->id_];
    for (auto const& fp : from->footpaths_) {
      if (fp.duration_ == walk_duration && is_target_station(fp.to_station_)) {
        return fp.to_station_;
      }
    }

    return -1;
  }

  void extract_journey(csa_journey& j) {
    (void)trip_reachable_;  // TODO(root): Delete when unnecessary
    // TODO(root): Only implemented for Dir == search_dir::FWD
    if (j.arrival_time_ ==
        j.start_time_ + final_footpaths_[j.start_station_->id_]) {
      // Just walking from source stop is optimal
      auto meta_target = find_meta_target(j.start_station_);
      j.transfers_ = 0;  // Don't know if this is necessary
      j.edges_.emplace_back(j.start_station_, &tt_.stations_[meta_target],
                            j.start_time_, j.arrival_time_);
    } else {
      auto const arrival = j.arrival_time_;
      auto const departure = j.start_time_;
      auto* stop = j.start_station_;
      auto transfers = j.transfers_;
      for (; transfers > 0; --transfers) {
        auto jp = get_journey_pointer(*stop, transfers, arrival, departure);

        stop = &tt_.stations_[jp.exit_con_->from_station_];
      }
    }
  }

  template <typename ArrDepList>  // Temporary template badness
  auto get_fitting_arrival(ArrDepList const& arrival_time, time arrival,
                           time departure_limit, unsigned transfers) {
    // find earliest departure which arrives at the same time as the current
    // journey
    auto it = get_pair_departing_after<Dir>(arrival_time, departure_limit);
    for (; it != arrival_time.end(); ++it) {
      if (it->second[transfers] == arrival) {
        return it;
      }
    }

    return arrival_time.end();
  }

  journey_pointer get_journey_pointer(csa_station const& station,
                                      unsigned transfers, time const arrival,
                                      time const departure) {
    // TODO(root): BIDIR
    for (auto const& fp : station.footpaths_) {
      if (fp.to_station_ == fp.from_station_) {
        continue;
      } else {
        auto const& arrival_time = arrival_time_[fp.to_station_];
        if (auto it = get_fitting_arrival(arrival_time, arrival,
                                          departure + fp.duration_, transfers);
            it != arrival_time.end()) {
          // found next target -> now we can search for the next connection from
          // there
          auto* const enter_station = &tt_.stations_[fp.to_station_];
          auto const new_departure = it->first;

          (void)new_departure;  // TODO(root): Delete when unnecessary

          // TODO(root): make similar function to get_exit_candidates()
          for (auto const& enter_con : enter_station->outgoing_connections_) {
            for (auto const& exit_con :
                 tt_.trip_to_connections_[enter_con->trip_]) {
              auto const& con_arrival_time =
                  arrival_time_[exit_con->to_station_];
              // TODO(root): find departure limit (transfers may be wrong too)
              if (auto exit_it = get_fitting_arrival(con_arrival_time, arrival,
                                                     -1, transfers - 1);
                  exit_it != con_arrival_time.end()) {
                return {enter_con, exit_con, &fp};
              }
            }
          }
        }
      }
    }

    return {};
  }

  csa_timetable const& tt_;
  std::map<station_id, time> const& start_times_;
  TargetStations const& target_stations_;

  ArrivalTimes const& arrival_time_;
  TripReachable const& trip_reachable_;
  FinalFootpaths const& final_footpaths_;
};

}  // namespace motis::csa