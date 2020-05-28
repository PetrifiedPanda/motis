#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "utl/to_vec.h"

namespace motis::loader::gtfs {

template <typename T>
struct flat_map {
  using index_t = std::size_t;
  using entry_t = std::pair<index_t, T>;
  using iterator = typename std::vector<entry_t>::iterator;
  using const_iterator = typename std::vector<entry_t>::const_iterator;

  struct cmp {
    bool operator()(entry_t const& lhs, entry_t const& rhs) {
      return lhs.first < rhs.first;
    }
  };

  std::vector<T> to_vector() {
    return utl::to_vec(elements_, [](entry_t const& el) { return el.second; });
  }

  template <typename... Args>
  void emplace(index_t idx, Args... args) {
    auto s = std::make_pair(idx, T(std::forward<Args>(args)...));
    auto it = std::lower_bound(elements_.begin(), elements_.end(), s, cmp());
    elements_.emplace(it, std::move(s));
  }

  T& operator[](index_t idx) {
    auto s = std::make_pair(idx, T());
    auto it = std::lower_bound(elements_.begin(), elements_.end(), s, cmp());
    if (it == elements_.end() || it->first != idx) {
      it = elements_.emplace(it, s);
    }
    return it->second;
  }

  T& front() { return elements_[0]; }
  T& back() { return elements_[elements_.size() - 1]; }
  T const& front() const { return elements_[0]; }
  T const& back() const { return elements_[elements_.size() - 1]; }

  iterator begin() { return elements_.begin(); }
  iterator end() { return elements_.end(); }
  const_iterator begin() const { return elements_.begin(); }
  const_iterator end() const { return elements_.end(); }
  friend const_iterator begin(flat_map const& m) { return m.begin(); }
  friend const_iterator end(flat_map const& m) { return m.end(); }
  friend iterator begin(flat_map& m) { return m.begin(); }
  friend iterator end(flat_map& m) { return m.end(); }

private:
  std::vector<entry_t> elements_;
};

}  // namespace motis::loader::gtfs
