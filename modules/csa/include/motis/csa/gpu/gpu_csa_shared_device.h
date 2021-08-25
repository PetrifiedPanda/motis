#pragma once

#include "motis/csa/gpu/dep_arr_vec.h"

extern "C" {

__host__ __device__ inline int divup(int a, int b) {
  return ((a % b) != 0) ? (a / b + 1) : (a / b);
}

__device__ dep_arr_vec create_dep_arr_vec(
    size_t init_size = 0, dep_arr_pair init_value = CSA_CUDA_INVALID_PAIR);

__device__ void free_dep_arr_vec(dep_arr_vec& vec);

__device__ void dep_ar_vec_push_front(dep_arr_vec& vec,
                                      dep_arr_pair const& key);

__device__ void dep_ar_vec_push_back(dep_arr_vec& vec, dep_arr_pair const& key);

__device__ void dep_ar_vec_insert(dep_arr_vec& vec, size_t location,
                                  dep_arr_pair const& key);
}  // extern "C"