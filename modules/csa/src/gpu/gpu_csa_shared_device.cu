#include "motis/csa/gpu/gpu_csa_shared_device.h"

#include <cassert>
#include <cstdio>

#include <cuda.h>
#include <cuda_runtime_api.h>

extern "C" {

__device__ void init_arr(dep_arr_pair* arr, size_t start, size_t end,
                         dep_arr_pair const& init_value) {
  for (size_t i = start; i < end; ++i) {
    arr[i] = init_value;
  }
}

__device__ dep_arr_vec create_dep_arr_vec(size_t init_size,
                                          dep_arr_pair init_value) {
  dep_arr_vec result;
  if (init_size != 0) {
    result.data_ = new dep_arr_pair[init_size];
    result.size_ = result.capacity_ = init_size;

    init_arr(result.data_, 0, init_size, init_value);
  }

  return {nullptr, 0, 0};
}

__device__ void free_dep_arr_vec(dep_arr_vec& vec) { delete[] vec.data_; }

__device__ void change_allocation_size(dep_arr_vec& vec, size_t new_capacity,
                                       size_t new_size,
                                       dep_arr_pair const& init_value) {
  dep_arr_pair* old_data = vec.data_;
  vec.data_ = new dep_arr_pair[new_capacity];

  for (size_t i = 0; i < vec.size_; ++i) {
    vec.data_[i] = old_data[i];
  }

  init_arr(vec.data_, vec.size_, new_size, init_value);

  vec.size_ = new_size;
  vec.capacity_ = new_capacity;

  delete[] old_data;
}

__device__ void resize(dep_arr_vec& vec, size_t new_size,
                       dep_arr_pair const& init_value) {
  if (new_size <= vec.capacity_) {
    init_arr(vec.data_, vec.size_, new_size, init_value);
    vec.size_ = new_size;
  } else {
    change_allocation_size(vec, new_size, new_size, init_value);
  }
}

__device__ void shift_one_front(dep_arr_vec& vec, size_t start) {
  size_t limit = start - 1;
  for (size_t i = vec.size_; i != limit; --i) {
    vec.data_[i] = vec.data_[i - 1];
  }
  ++vec.size_;
}

__device__ void dep_ar_vec_push_front(dep_arr_vec& vec,
                                      dep_arr_pair const& key) {
  if (vec.size_ == vec.capacity_) {
    size_t new_capacity = vec.size_ + vec.size_ / 2;
    change_allocation_size(vec, new_capacity, vec.size_, key);
  }

  shift_one_front(vec, 0);
  vec.data_[0] = key;
}

__device__ void dep_ar_vec_push_back(dep_arr_vec& vec,
                                     dep_arr_pair const& key) {
  if (vec.size_ < vec.capacity_) {
    vec.data_[vec.size_] = key;
    ++vec.size_;
  } else {
    size_t new_capacity = vec.size_ + vec.size_ / 2;
    change_allocation_size(vec, new_capacity, vec.size_ + 1, key);
  }
}

__device__ void dep_ar_vec_insert(dep_arr_vec& vec, size_t location,
                                  dep_arr_pair const& key) {
  if (vec.size_ == vec.capacity_) {
    size_t new_capacity = vec.size_ + vec.size_ / 2;
    change_allocation_size(vec, new_capacity, vec.size_, key);
  }

  shift_one_front(vec, location);
  vec.data_[location] = key;
}
};