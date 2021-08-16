#pragma once

#include "motis/csa/gpu/dep_arr_vec.h"

extern "C" {

struct gpu_csa_profile_result {
  dep_arr_vec* arrival_times_;
  arrival_times* trip_reachable_;
  gpu_csa_time* final_footpaths_;
};

gpu_csa_profile_result gpu_csa_profile_search(
    gpu_timetable* tt, gpu_csa_start* starts, uint32_t num_starts,
    uint32_t num_queries, uint32_t start_bucket, gpu_csa_time time_limit);

void free_gpu_csa_profile_result(gpu_csa_profile_result& result,
                                 size_t num_stations);

}  // extern "C"
