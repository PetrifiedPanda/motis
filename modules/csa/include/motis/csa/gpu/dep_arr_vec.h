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

constexpr dep_arr_pair DEFAULT_PAIR = {CUDA_INVALID_TIME, {CUDA_INVALID_TIME}};

__device__ dep_arr_vec create_dep_arr_vec(
    size_t init_size = 0, dep_arr_pair const& init_value = DEFAULT_PAIR);

__device__ void free_dep_arr_vec(dep_arr_vec& vec);

__device__ void dep_ar_vec_push_front(dep_arr_vec& vec,
                                      dep_arr_pair const& key);

__device__ void dep_ar_vec_push_back(dep_arr_vec& vec, dep_arr_pair const& key);

__device__ void dep_ar_vec_insert(dep_arr_vec& vec, size_t location,
                                  dep_arr_pair const& key);
};
