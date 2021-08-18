#pragma once

#include "motis/csa/gpu/gpu_csa_shared.h"

extern "C" {

struct gpu_csa_result {
  gpu_csa_time* station_arrivals_;
  gpu_csa_con_idx* trip_reachable_;
};

struct gpu_csa_start {
  uint32_t query_idx_;
  gpu_csa_station_id station_idx_;
  gpu_csa_time start_time_;
};

gpu_csa_result gpu_csa_search(struct gpu_timetable*, struct gpu_csa_start*,
                              uint32_t num_starts, uint32_t num_queries,
                              uint32_t start_bucket, gpu_csa_time time_limit);

void gpu_csa_free_result(gpu_csa_result*);

}  // extern "C"
