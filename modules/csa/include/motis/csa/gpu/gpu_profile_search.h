#pragma once

#include "gpu_csa_profile.h"

#include "motis/core/common/raii.h"
#include "motis/csa/csa_profile_reconstruction.h"

namespace motis::csa {

template <typename T>
struct profile_wrapper {
  profile_wrapper(T* ptr, size_t size) : ptr_{ptr}, size_{size} {}

  const T& operator[](size_t i) const { return ptr_[i]; }

  const T* begin() const { return ptr_; }

  const T* end() const { return ptr_ + size_; }

  T* ptr_;
  size_t size_;
};

/*
 * This class is made to have the same memory layout as dep_arr_vec, just with
 * an iterator function we cannot add in extern "C" blocks, so the
 * reconstruction works
 */
struct imposter_dep_arr_vec {

  const dep_arr_pair* begin() const { return data_; }

  const dep_arr_pair* end() const { return data_ + size_; }

  dep_arr_pair* data_;
  size_t size_;
  size_t capacity_;
};

/*
 * Assert sizes statically so any changes that make reinterpret_casting between
 * imposter_dep_arr_vec and dep_arr_vec will not compile
 */
static_assert(sizeof(imposter_dep_arr_vec::data_) ==
              sizeof(dep_arr_vec::data_));
static_assert(sizeof(imposter_dep_arr_vec::size_) ==
              sizeof(dep_arr_vec::size_));
static_assert(sizeof(imposter_dep_arr_vec::capacity_) ==
              sizeof(dep_arr_vec::capacity_));

static_assert(sizeof(imposter_dep_arr_vec) == sizeof(dep_arr_vec));

struct gpu_profile_search {
  static constexpr time INVALID = std::numeric_limits<time>::max();

  gpu_profile_search(schedule const& sched, csa_timetable const& tt,
                     csa_query const& q, csa_statistics& stats)
      : sched_{sched}, tt_{tt}, q_{q}, stats_{stats} {}

  template <typename Results>
  void search_in_interval(Results& results, interval const& search_interval,
                          bool ontrip_at_interval_end) {
    search(results, collect_start_times(tt_, q_, search_interval,
                                        ontrip_at_interval_end));
  }

  template <typename Results>
  void search(Results& results, std::set<motis::time> const& start_times) {
    if (start_times.empty()) {
      return;  // TODO(root): do this?
    }

    // TODO(root): Assign
    gpu_csa_time* final_footpaths = nullptr;
    std::vector<station_id> target_stations;
    std::map<station_id, time> starts;

    MOTIS_START_TIMING(search_timing);
    auto res = ::gpu_csa_profile_search(tt_.gpu_timetable_.ptr_, nullptr, 0, 0,
                                        0, 0, final_footpaths);
    MOTIS_STOP_TIMING(search_timing);
    if (res.arrival_times_ == nullptr || res.trip_reachable_ == nullptr ||
        res.final_footpaths_ == nullptr) {
      delete[] final_footpaths;
      throw std::runtime_error{"Error in GPU code"};
    }
    MOTIS_FINALLY(
        [&]() { free_gpu_csa_profile_result(res, tt_.stations_.size()); })

    MOTIS_START_TIMING(reconstruction_timing);
    auto const station_count = tt_.stations_.size();
    auto recon =
        csa_profile_reconstruction<search_dir::FWD, std::vector<station_id>,
                                   profile_wrapper<imposter_dep_arr_vec>,
                                   profile_wrapper<::arrival_times>,
                                   profile_wrapper<gpu_csa_time>>(
            tt_, starts, target_stations,
            profile_wrapper(
                reinterpret_cast<imposter_dep_arr_vec*>(res.arrival_times_),
                station_count),
            profile_wrapper(res.trip_reachable_, tt_.trip_count_),
            profile_wrapper(res.final_footpaths_, station_count));
    MOTIS_STOP_TIMING(reconstruction_timing);
  }

  schedule const& sched_;
  csa_timetable const& tt_;
  csa_query const& q_;
  csa_statistics& stats_;
};

}  // namespace motis::csa