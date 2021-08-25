#include "motis/csa/gpu/gpu_csa_profile.h"

#include <cassert>
#include <cstdio>

#include <cuda_device_runtime_api.h>
#include <cuda_runtime_api.h>

#include "motis/csa/gpu/dep_arr_vec.h"
#include "motis/csa/gpu/gpu_csa.h"

extern "C" {

// TODO(root): BAD copied function
__host__ __device__ inline int divup(int a, int b) {
  return ((a % b) != 0) ? (a / b + 1) : (a / b);
}

__device__ inline int get_array_index() {  // TODO(root): better name
  return blockIdx.x * blockDim.x + threadIdx.x;
}

struct d_profile_query {
  uint32_t arrival_times_size_;
  uint32_t trip_reachable_size_;
  uint32_t final_footpaths_size_;
  uint32_t start_bucket_;
  uint32_t num_queries_;

  dep_arr_vec* arrival_times_;
  arrival_times* trip_reachable_;
  gpu_csa_time* final_footpaths_;

  gpu_timetable* tt_;
};

__global__ void free_device_arrivals_kernel(dep_arr_vec* arrival_times,
                                            size_t station_count) {
  auto const i = get_array_index();
  if (i < station_count) {
    delete[] arrival_times[i].data_;
    arrival_times[i].data_ = nullptr;
  }
}

constexpr auto THREADS_PER_BLOCK = 1024;

void free_d_profile_query(d_profile_query& q) {
  cudaError_t code;

loop:
  free_device_arrivals_kernel<<<divup(q.arrival_times_size_, THREADS_PER_BLOCK),
                                THREADS_PER_BLOCK>>>(q.arrival_times_,
                                                     q.arrival_times_size_);
  CUDA_CALL(cudaGetLastError())
  CUDA_CALL(cudaDeviceSynchronize())
  goto success;

  // TODO(root): this may cause an infinite loop and/or memory leaks later :)
fail:
  goto loop;
success:
  cudaFree(q.arrival_times_);
  cudaFree(q.trip_reachable_);
  cudaFree(q.final_footpaths_);
  cudaFree(q.tt_);

  q.arrival_times_ = nullptr;
  q.trip_reachable_ = nullptr;
  q.final_footpaths_ = nullptr;
  q.tt_ = nullptr;
}

__global__ void init_arrival_times_kernel(dep_arr_vec* arrival_times,
                                          uint32_t size) {
  auto const i = get_array_index();
  if (i < size) {
    arrival_times[i] = create_dep_arr_vec(1);
  }
}

__global__ void init_trip_reachable_kernel(arrival_times* trip_reachable,
                                           uint32_t size) {
  auto const i = get_array_index();
  if (i < size) {
    for (auto j = 0; j <= GPU_CSA_MAX_TRANSFERS; ++j) {
      trip_reachable[i][j] = CUDA_INVALID_TIME;
    }
  }
}

__global__ void gpu_csa_profile_kernel(d_profile_query q) {
  init_arrival_times_kernel<<<divup(q.arrival_times_size_, THREADS_PER_BLOCK),
                              THREADS_PER_BLOCK>>>(q.arrival_times_,
                                                   q.arrival_times_size_);
  init_trip_reachable_kernel<<<divup(q.trip_reachable_size_, THREADS_PER_BLOCK),
                               THREADS_PER_BLOCK>>>(q.trip_reachable_,
                                                    q.trip_reachable_size_);
}

bool copy_back_arrival_times(dep_arr_vec* arrival_times,
                             dep_arr_vec* d_arrival_times,
                             uint32_t station_count, size_t num_bytes) {
  cudaError_t code;

  CUDA_CALL(cudaMemcpy(arrival_times, d_arrival_times, num_bytes,
                       cudaMemcpyDeviceToHost))
  for (auto i = 0; i < station_count; ++i) {
    auto& vec = arrival_times[i];
    auto const* d_data = vec.data_;

    if (vec.data_ != nullptr) {
      vec.capacity_ = vec.size_;
      vec.data_ = new dep_arr_pair[vec.size_];

      CUDA_CALL(cudaMemcpy(vec.data_, d_data, sizeof(dep_arr_pair) * vec.size_,
                           cudaMemcpyDeviceToHost))
    }
  }

  return true;

fail:
  return false;
}

gpu_csa_profile_result gpu_csa_profile_search(
    gpu_timetable* tt, gpu_csa_start* starts, uint32_t num_starts,
    uint32_t num_queries, uint32_t start_bucket, gpu_csa_time time_limit,
    gpu_csa_time* final_footpaths) {
  assert(final_footpaths != nullptr);

  // For macros in gpu_csa_shared.h
  size_t device_bytes = 0U;
  cudaError_t code;

  auto const station_count = tt->station_count_;
  auto const trip_count = tt->trip_count_;

  // Allocate result
  gpu_csa_profile_result r;
  r.arrival_times_ = new dep_arr_vec[station_count];
  r.trip_reachable_ = new arrival_times[trip_count];
  r.final_footpaths_ = final_footpaths;

  d_profile_query q;
  q.arrival_times_size_ = station_count;
  q.trip_reachable_size_ = trip_count;
  q.final_footpaths_size_ = station_count;
  q.start_bucket_ = start_bucket;
  q.num_queries_ = num_queries;
  q.arrival_times_ = nullptr;
  q.trip_reachable_ = nullptr;
  q.final_footpaths_ = nullptr;

  auto const arr_time_num_bytes = station_count * sizeof(dep_arr_vec);
  auto const trip_reachable_num_bytes = trip_count * sizeof(arrival_times);

  // Allocate Data structures on device
  CUDA_CALL(cudaMalloc(&q.arrival_times_, arr_time_num_bytes))
  CUDA_CALL(cudaMalloc(&q.trip_reachable_, trip_reachable_num_bytes))
  CUDA_COPY_TO_DEVICE(gpu_csa_time, q.final_footpaths_, r.final_footpaths_,
                      station_count)

  // DO ALGORITHM
  gpu_csa_profile_kernel<<<1, 1>>>(q);
  CUDA_CALL(cudaGetLastError())
  CUDA_CALL(cudaDeviceSynchronize())

  // Copy back results
  if (!copy_back_arrival_times(r.arrival_times_, q.arrival_times_,
                               station_count, arr_time_num_bytes)) {
    goto fail;
  }

  CUDA_CALL(cudaMemcpy(r.trip_reachable_, q.trip_reachable_,
                       trip_reachable_num_bytes, cudaMemcpyDeviceToHost))
  // final_footpaths remains unchanged in the algorithm

  free_d_profile_query(q);

  return r;
fail:
  free_gpu_csa_profile_result(r, station_count);
  free_d_profile_query(q);

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