#include "motis/csa/gpu/gpu_csa_shared.h"

#include <cstdio>

#define FMT_HUMAN_READABLE "%.1f%s"
#define HUMAN_READABLE(size)                                             \
  ((size) > 1024 * 1024 * 1024) ? (((float)(size)) / 1024 / 1024 / 1024) \
  : ((size) > 1024 * 1024)      ? (((float)(size)) / 1024 / 1024)        \
  : ((size) > 1024)             ? (((float)(size)) / 1024)               \
                                : ((float)(size)),                                   \
      ((size) > 1024 * 1024 * 1024) ? "GB"                               \
      : ((size) > 1024 * 1024)      ? "MB"                               \
      : ((size) > 1024)             ? "kb"                               \
                                    : "b"

//==============================================================================
// TIMETABLE
//------------------------------------------------------------------------------

gpu_timetable* create_csa_gpu_timetable(
    gpu_csa_con* conns, uint32_t* bucket_starts, uint32_t bucket_count,
    uint32_t conn_count, uint32_t station_count, uint32_t trip_count) {
  size_t device_bytes = 0U;

  cudaError_t code;
  gpu_timetable* tt =
      static_cast<gpu_timetable*>(malloc(sizeof(gpu_timetable)));

  tt->station_count_ = station_count;
  tt->trip_count_ = trip_count;
  tt->bucket_count_ = bucket_count;
  tt->conns_ = nullptr;

  CUDA_COPY_TO_DEVICE(uint32_t, tt->bucket_starts_, bucket_starts,
                      bucket_count);
  CUDA_COPY_TO_DEVICE(struct gpu_csa_con, tt->conns_, conns, conn_count);

  printf("Schedule size on GPU: " FMT_HUMAN_READABLE "\n",
         HUMAN_READABLE(device_bytes));

  return tt;

fail:
  if (tt != nullptr) {
    cudaFree(tt->conns_);
  }
  free(tt);
  return nullptr;
}

void free_csa_gpu_timetable(gpu_timetable* tt) {
  if (tt == nullptr) {
    return;
  }
  cudaFree(tt->conns_);
  tt->conns_ = nullptr;
  tt->station_count_ = 0U;
  tt->trip_count_ = 0U;
  free(tt);
}