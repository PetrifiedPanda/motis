#pragma once

#include "motis/csa/gpu/gpu_csa.h"

extern "C" {

using arrival_times = gpu_csa_time[GPU_CSA_MAX_TRANSFERS + 1];

struct dep_arr_pair {
  gpu_csa_time departure_;
  arrival_times arrivals_;
};

struct dep_arr_vec {
  dep_arr_pair* data_;
  size_t size_;
  size_t capacity_;
};

constexpr gpu_csa_time CUDA_INVALID_TIME = UINT16_MAX;

constexpr dep_arr_pair CSA_CUDA_INVALID_PAIR = {CUDA_INVALID_TIME,
                                                {CUDA_INVALID_TIME}};
}  // extern "C"
