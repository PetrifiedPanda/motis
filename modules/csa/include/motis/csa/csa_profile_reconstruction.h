#pragma once

#include "motis/csa/csa_reconstruction.h"

namespace motis::csa {

template <search_dir Dir, typename ArrivalTimes, typename TripReachable>
struct csa_profile_reconstruction {
  csa_profile_reconstruction(csa_timetable const& tt,
                             std::map<station_id, time> const& start_times,
                             ArrivalTimes const& arrival_time,
                             TripReachable const& trip_reachable)
      : tt_(tt),
        start_times_(start_times),
        arrival_time_(arrival_time),
        trip_reachable_(trip_reachable) {}

  void extract_journey(csa_journey& j) {
    // TODO(root):
  }

  csa_timetable const& tt_;
  std::map<station_id, time> const& start_times_;
  ArrivalTimes const& arrival_time_;
  TripReachable const& trip_reachable_;
};

};  // namespace motis::csa