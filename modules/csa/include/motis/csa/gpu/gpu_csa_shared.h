#pragma once

#include <inttypes.h>

#define CUDA_CALL(call)                                   \
  if ((code = call) != cudaSuccess) {                     \
    printf("CUDA error: %s at " STR(call) " %s:%d\n",     \
           cudaGetErrorString(code), __FILE__, __LINE__); \
    goto fail;                                            \
  }

extern "C" {

constexpr auto GPU_CSA_MAX_TRANSFERS = 7U;
constexpr auto GPU_CSA_MAX_TRAVEL_TIME = 1440U;

typedef uint16_t gpu_csa_time;
typedef uint32_t gpu_csa_station_id;
typedef uint32_t gpu_csa_trip_idx;
typedef uint16_t gpu_csa_con_idx;

struct gpu_csa_con {
  gpu_csa_station_id from_, to_;
  gpu_csa_trip_idx trip_;
  gpu_csa_time dep_, arr_;
  gpu_csa_con_idx trip_con_idx_;
  bool in_allowed_, out_allowed_;
};

struct gpu_timetable {
  struct gpu_csa_con* conns_;
  uint32_t* bucket_starts_;
  uint32_t station_count_, trip_count_, bucket_count_;
};

gpu_timetable* create_csa_gpu_timetable(
    gpu_csa_con* conns, uint32_t* bucket_starts, uint32_t bucket_count,
    uint32_t conn_count, uint32_t station_count, uint32_t trip_count);

void free_csa_gpu_timetable(gpu_timetable* tt);

}  // extern "C"
