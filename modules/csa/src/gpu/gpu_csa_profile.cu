#include "motis/csa/gpu/gpu_csa_profile.h"

#include <cstdio>

#include <cuda_device_runtime_api.h>
#include <cuda_runtime_api.h>

#include "motis/csa/gpu/dep_arr_vec.h"
#include "motis/csa/gpu/gpu_csa.h"

extern "C" {

__global__ void gpu_csa_profile_kernel() {
  // TODO(root):
}

// TODO(root): not sure if it would be worth it to parallelize this
__global__ void free_device_arrivals_kernel(dep_arr_vec* arrival_times,
                                            size_t station_count) {
  for (size_t i = 0; i < station_count; ++i) {
    delete[] arrival_times[i].data_;
  }
}

gpu_csa_profile_result gpu_csa_profile_search(
    gpu_timetable* tt, gpu_csa_start* starts, uint32_t num_starts,
    uint32_t num_queries, uint32_t start_bucket, gpu_csa_time time_limit) {
  // Variables for macros in gpu_csa_shared.h
  cudaError_t code;
  size_t device_bytes = 0U;

  auto const station_count = tt->station_count_;
  gpu_csa_profile_result r;
  r.arrival_times_ = new dep_arr_vec[station_count];
  r.trip_reachable_ = new arrival_times[tt->trip_count_];
  r.final_footpaths_ = new gpu_csa_time[station_count];

  size_t num_bytes = station_count * sizeof(dep_arr_vec);
  dep_arr_vec* d_arrival_times = nullptr;

  for (auto i = 0; i < station_count; ++i) {
    r.arrival_times_[i] = {nullptr, 0, 0};
  }
  CUDA_COPY_TO_DEVICE(dep_arr_vec, d_arrival_times, r.arrival_times_,
                      station_count)

  // DO ALGORITHM

  CUDA_CALL(cudaMemcpy(r.arrival_times_, d_arrival_times, num_bytes,
                       cudaMemcpyDeviceToHost))
  for (auto i = 0; i < station_count; ++i) {
    auto const* d_ptr = r.arrival_times_[i].data_;
    auto& vec = r.arrival_times_[i];

    auto const byte_size = sizeof(dep_arr_pair) * vec.size_;

    vec.capacity_ = vec.size_;
    vec.data_ = new dep_arr_pair[vec.size_];  // Old data still in device memory

    CUDA_CALL(cudaMemcpy(vec.data_, d_ptr, byte_size, cudaMemcpyDeviceToHost))
  }

  free_device_arrivals_kernel<<<1, 1>>>(d_arrival_times, station_count);
  CUDA_CALL(cudaGetLastError())
  CUDA_CALL(cudaDeviceSynchronize())

  cudaFree(d_arrival_times);

  return r;
fail:
  free_gpu_csa_profile_result(r, station_count);
  if (d_arrival_times) {
    cudaFree(d_arrival_times);
  }

  return {nullptr, nullptr, nullptr};
}

void free_gpu_csa_profile_result(gpu_csa_profile_result& result,
                                 size_t num_stations) {
  for (size_t i = 0; i < num_stations; ++i) {
    delete[] result.arrival_times_[i].data_;
  }

  delete[] result.arrival_times_;
  delete[] result.trip_reachable_;
  delete[] result.final_footpaths_;

  result.arrival_times_ = nullptr;
  result.trip_reachable_ = nullptr;
  result.final_footpaths_ = nullptr;
}

}  // extern "C"