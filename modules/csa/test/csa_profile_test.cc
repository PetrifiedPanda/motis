#include "gtest/gtest.h"

#include <algorithm>
#include <array>
#include <unordered_map>

#include "motis/core/schedule/station.h"
#include "motis/csa/cpu/csa_profile_search_default_cpu.h"

using namespace motis;
using namespace motis::csa;
using namespace motis::csa::cpu;

/*
 * This test does not account for interstop footpaths and many other features of
 * the algorithm but it tests the basic features of the algorithm. The graph can
 * be found in the paper on page 24
 */
struct simple_profile : ::testing::Test {
public:
  static csa_station create_station(
      csa_station const& tmp_stat, unsigned id, motis::time transfer_time,
      std::vector<csa_connection const*>&& outgoing_connections,
      std::vector<csa_connection const*>&& incoming_connections) {
    auto stat = tmp_stat;
    stat.station_ptr_ = nullptr;
    stat.id_ = id;
    stat.transfer_time_ = transfer_time;
    // not sure if this is necessary
    stat.footpaths_ = {footpath{id, id, 0}};
    stat.incoming_footpaths_ = {footpath{id, id, 0}};

    stat.outgoing_connections_ = std::move(outgoing_connections);
    stat.incoming_connections_ = std::move(incoming_connections);
    return stat;
  }

  csa_connection make_simple_con(uint32_t from, uint32_t to, motis::time dep,
                                 motis::time arr) {
    constexpr auto SERVICE_CLASS = service_class::IC;
    return csa_connection(from, to, dep, arr, 0, trip_id_++, 0, true, true,
                          SERVICE_CLASS, &dummy_light_con_);
  }

protected:
  void SetUp() override {
    tt_.stations_ = {};
    tt_.fwd_connections_ = {};
    tt_.bwd_connections_ = {};

    tt_.fwd_connections_.reserve(CONN_COUNT);
    tt_.fwd_connections_.push_back(
        make_simple_con(char_to_id_['s'], char_to_id_['v'], 5, 8));
    tt_.fwd_connections_.push_back(
        make_simple_con(char_to_id_['s'], char_to_id_['x'], 6, 7));
    tt_.fwd_connections_.push_back(
        make_simple_con(char_to_id_['s'], char_to_id_['z'], 7, 8));
    tt_.fwd_connections_.push_back(
        make_simple_con(char_to_id_['x'], char_to_id_['y'], 8, 9));
    tt_.fwd_connections_.push_back(
        make_simple_con(char_to_id_['x'], char_to_id_['t'], 8, 13));
    tt_.fwd_connections_.push_back(
        make_simple_con(char_to_id_['z'], char_to_id_['t'], 9, 12));
    tt_.fwd_connections_.push_back(
        make_simple_con(char_to_id_['v'], char_to_id_['t'], 9, 14));
    tt_.fwd_connections_.push_back(
        make_simple_con(char_to_id_['y'], char_to_id_['t'], 10, 11));
    tt_.bwd_connections_ = tt_.fwd_connections_;
    std::sort(tt_.bwd_connections_.begin(), tt_.bwd_connections_.end(),
              [](auto const& c1, auto const& c2) {
                return c1.arrival_ > c2.arrival_;
              });

    // sanity check that all conns are in separate trips
    for (auto const& con1 : tt_.fwd_connections_) {
      for (auto const& con2 : tt_.fwd_connections_) {
        if (&con1 == &con2) {
          continue;
        }
        ASSERT_NE(con1.trip_, con2.trip_);
      }
    }

    ASSERT_EQ(tt_.fwd_connections_.size(), tt_.bwd_connections_.size());
    // Check if cons are sorted by ascending departure and descending arrival
    for (auto i = 1; i < tt_.fwd_connections_.size(); ++i) {
      ASSERT_LE(tt_.fwd_connections_[i - 1].departure_,
                tt_.fwd_connections_[i].departure_);
      ASSERT_GE(tt_.bwd_connections_[i - 1].arrival_,
                tt_.bwd_connections_[i].arrival_);
    }

    ASSERT_EQ(tt_.fwd_connections_.size(), CONN_COUNT);

    tt_.stations_.reserve(STATION_COUNT);
    tt_.trip_count_ = CONN_COUNT;
    station dummy_station;
    csa_station stat{&dummy_station};

    tt_.stations_.push_back(
        create_station(stat, 0, 0,
                       {&tt_.fwd_connections_[0], &tt_.fwd_connections_[1],
                        &tt_.fwd_connections_[2]},
                       {}));
    tt_.stations_.push_back(create_station(
        stat, 1, 0, {&tt_.fwd_connections_[6]}, {&tt_.fwd_connections_[0]}));
    tt_.stations_.push_back(
        create_station(stat, 2, 0, {},
                       {&tt_.fwd_connections_[4], &tt_.fwd_connections_[5],
                        &tt_.fwd_connections_[6], &tt_.fwd_connections_[7]}));
    tt_.stations_.push_back(create_station(
        stat, 3, 0, {&tt_.fwd_connections_[3], &tt_.fwd_connections_[4]},
        {&tt_.fwd_connections_[1]}));
    tt_.stations_.push_back(create_station(
        stat, 4, 0, {&tt_.fwd_connections_[7]}, {&tt_.fwd_connections_[3]}));
    tt_.stations_.push_back(create_station(
        stat, 5, 0, {&tt_.fwd_connections_[5]}, {&tt_.fwd_connections_[2]}));

    ASSERT_EQ(tt_.stations_.size(), STATION_COUNT);
    for (auto const& station : tt_.stations_) {
      for (auto const* con : station.outgoing_connections_) {
        ASSERT_EQ(station.id_, con->from_station_);
      }

      for (auto const* con : station.incoming_connections_) {
        ASSERT_EQ(station.id_, con->to_station_);
      }
    }

    tt_.trip_to_connections_.reserve(CONN_COUNT);
    for (auto i = 0; i < tt_.fwd_connections_.size(); ++i) {
      ASSERT_EQ(tt_.fwd_connections_[i].trip_, i);
      tt_.trip_to_connections_.push_back({&tt_.fwd_connections_[i]});
    }
    trip_id_ = 0;
  }

public:
  static constexpr auto STATION_COUNT = 6;
  static constexpr auto CONN_COUNT = 8;
  uint32_t trip_id_ = 0;
  csa_timetable tt_;
  std::unordered_map<char, station_id> char_to_id_{
      {'s', 0}, {'v', 1}, {'t', 2}, {'x', 3}, {'y', 4}, {'z', 5}};
  light_connection dummy_light_con_;
};

TEST(profile_dominates, arr_time_domination) {
  auto a1 = array_maker<motis::time, MAX_TRANSFERS + 1>::make_array(10);
  auto a2 = a1;
  ASSERT_FALSE(csa_profile_search<search_dir::FWD>::dominates(a1, a2));
  ASSERT_FALSE(csa_profile_search<search_dir::FWD>::dominates(a2, a1));
  ASSERT_FALSE(csa_profile_search<search_dir::BWD>::dominates(a1, a2));
  ASSERT_FALSE(csa_profile_search<search_dir::BWD>::dominates(a2, a1));

  a1[5] = 5;
  ASSERT_TRUE(csa_profile_search<search_dir::FWD>::dominates(a1, a2));
  ASSERT_TRUE(csa_profile_search<search_dir::BWD>::dominates(a2, a1));

  auto const sent_fwd = array_maker<motis::time, MAX_TRANSFERS + 1>::make_array(
      csa_profile_search<search_dir::FWD>::INVALID);
  ASSERT_TRUE(csa_profile_search<search_dir::FWD>::dominates(a1, sent_fwd));
  ASSERT_TRUE(csa_profile_search<search_dir::FWD>::dominates(a2, sent_fwd));

  auto const sent_bwd = array_maker<motis::time, MAX_TRANSFERS + 1>::make_array(
      csa_profile_search<search_dir::BWD>::INVALID);
  ASSERT_TRUE(csa_profile_search<search_dir::BWD>::dominates(a1, sent_bwd));
  ASSERT_TRUE(csa_profile_search<search_dir::BWD>::dominates(a2, sent_bwd));
}

TEST(profile_dominates, pair_domination) {
  auto p1 = std::make_pair(
      0, array_maker<motis::time, MAX_TRANSFERS + 1>::make_array(10));
  auto p2 = p1;
  ASSERT_FALSE(csa_profile_search<search_dir::FWD>::dominates(p1, p2));
  ASSERT_FALSE(csa_profile_search<search_dir::FWD>::dominates(p2, p1));
  ASSERT_FALSE(csa_profile_search<search_dir::BWD>::dominates(p1, p2));
  ASSERT_FALSE(csa_profile_search<search_dir::BWD>::dominates(p2, p1));

  p1.second[5] = 5;
  ASSERT_TRUE(csa_profile_search<search_dir::FWD>::dominates(p1, p2));
  ASSERT_TRUE(csa_profile_search<search_dir::BWD>::dominates(p2, p1));
}

template <typename ArrTimes>
void check_all_val(ArrTimes const& times, motis::time val) {
  for (auto const& entry : times) {
    EXPECT_EQ(entry, val);
  }
}

template <typename Search>
void last_arr_time_pair_invalid(Search const& search) {
  for (auto const& arr_time_lst : search.arrival_time_) {
    EXPECT_EQ(arr_time_lst.back().first, search.INVALID);
    check_all_val(arr_time_lst.back().second, search.INVALID);
  }
}

template <typename Search>
void check_final_footpaths(Search const& search, uint32_t target) {
  for (auto i = 0; i < search.final_footpaths_.size(); ++i) {
    auto const val = search.final_footpaths_[i];
    if (i == target) {
      EXPECT_EQ(val, 0);
    } else {
      EXPECT_EQ(val, std::numeric_limits<motis::time>::max());
    }
  }
}

TEST_F(simple_profile, simple_fwd) {

  csa_statistics stats;
  csa_profile_search<search_dir::FWD> search{tt_, interval{0, 1000}, stats};
  search.add_dest(tt_.stations_[char_to_id_['t']]);

  search.search();

  check_final_footpaths(search, char_to_id_['t']);

  constexpr auto ARR_SIZE = MAX_TRANSFERS + 1;
  // ARRIVAL TIMES

  last_arr_time_pair_invalid(search);

  auto y_arr = search.arrival_time_[char_to_id_['y']];
  ASSERT_EQ(y_arr.size(), 2);
  EXPECT_EQ(y_arr.front().first, 10);
  check_all_val(y_arr.front().second, 11);

  auto z_arr = search.arrival_time_[char_to_id_['z']];
  ASSERT_EQ(z_arr.size(), 2);
  EXPECT_EQ(z_arr.front().first, 9);
  check_all_val(z_arr.front().second, 12);

  auto v_arr = search.arrival_time_[char_to_id_['v']];
  ASSERT_EQ(v_arr.size(), 2);
  EXPECT_EQ(v_arr.front().first, 9);
  check_all_val(v_arr.front().second, 14);

  auto x_arr = search.arrival_time_[char_to_id_['x']];
  ASSERT_EQ(x_arr.size(), 2);
  EXPECT_EQ(x_arr.front().first, 8);
  EXPECT_EQ(x_arr.front().second[0], 13);
  for (auto i = 1; i < ARR_SIZE; ++i) {
    EXPECT_EQ(x_arr.front().second[i], 11);
  }

  // The results for S come from page 25 of the paper
  auto s_arr = search.arrival_time_[char_to_id_['s']];
  ASSERT_EQ(s_arr.size(), 3);
  auto it = s_arr.begin();

  // first pair
  EXPECT_EQ(it->first, 6);
  EXPECT_EQ(it->second[0], search.INVALID);
  EXPECT_EQ(it->second[1], 12);
  for (auto i = 2; i < ARR_SIZE; ++i) {
    EXPECT_EQ(it->second[i], 11);
  }

  ++it;
  // second pair
  EXPECT_EQ(it->first, 7);
  EXPECT_EQ(it->second[0], search.INVALID);
  for (auto i = 1; i < ARR_SIZE; ++i) {
    EXPECT_EQ(it->second[i], 12);
  }

  // TRIP REACHABLE

  // 5 - 8
  EXPECT_EQ(search.trip_reachable_[0][0], search.INVALID);
  for (auto i = 1; i < ARR_SIZE; ++i) {
    EXPECT_EQ(search.trip_reachable_[0][i], 14);
  }

  // 6 - 7
  EXPECT_EQ(search.trip_reachable_[1][0], search.INVALID);
  EXPECT_EQ(search.trip_reachable_[1][1], 13);
  for (auto i = 2; i < ARR_SIZE; ++i) {
    EXPECT_EQ(search.trip_reachable_[1][i], 11);
  }

  // 7 - 8
  EXPECT_EQ(search.trip_reachable_[2][0], search.INVALID);
  for (auto i = 1; i < ARR_SIZE; ++i) {
    EXPECT_EQ(search.trip_reachable_[2][i], 12);
  }

  // 8 - 9
  EXPECT_EQ(search.trip_reachable_[3][0], search.INVALID);
  for (auto i = 1; i < ARR_SIZE; ++i) {
    EXPECT_EQ(search.trip_reachable_[3][i], 11);
  }

  // 8 - 13
  check_all_val(search.trip_reachable_[4], 13);

  // 9 - 12
  check_all_val(search.trip_reachable_[5], 12);

  // 9 - 14
  check_all_val(search.trip_reachable_[6], 14);

  // 10 - 11
  check_all_val(search.trip_reachable_[7], 11);

  auto res = search.get_results(tt_.stations_[char_to_id_['s']], false);

  // TODO(root): Test reconstruction results
}

TEST_F(simple_profile, simple_bwd) {
  csa_statistics stats;
  csa_profile_search<search_dir::BWD> search{tt_, interval{1000, 0}, stats};
  search.add_dest(tt_.stations_[char_to_id_['s']]);

  search.search();

  last_arr_time_pair_invalid(search);

  check_final_footpaths(search, char_to_id_['s']);

  // TODO(root): Actual testing
}