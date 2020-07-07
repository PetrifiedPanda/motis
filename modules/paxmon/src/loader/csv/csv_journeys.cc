#include "motis/paxmon/loader/csv/csv_journeys.h"

#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <optional>
#include <regex>
#include <utility>

#include "fmt/ostream.h"

#include "utl/parser/buf_reader.h"
#include "utl/parser/csv_range.h"
#include "utl/parser/file.h"
#include "utl/parser/line_range.h"
#include "utl/pipes/for_each.h"

#include "motis/core/common/logging.h"
#include "motis/core/schedule/time.h"
#include "motis/core/access/station_access.h"
#include "motis/core/access/trip_iterator.h"
#include "motis/core/conv/trip_conv.h"

#include "motis/paxmon/util/get_station_idx.h"

using namespace motis::logging;
using namespace motis::paxmon::util;

namespace std {
inline std::ostream& operator<<(
    std::ostream& out,
    std::optional<std::pair<std::uint64_t, std::uint64_t>> const& id) {
  if (id) {
    return out << id->first << "." << id->second;
  } else {
    return out << "-";
  }
}
}  // namespace std

namespace motis::paxmon::loader::csv {

struct row {
  utl::csv_col<std::uint64_t, UTL_NAME("id")> id_;
  utl::csv_col<std::uint64_t, UTL_NAME("secondary_id")> secondary_id_;
  utl::csv_col<utl::cstr, UTL_NAME("leg_type")> leg_type_;
  utl::csv_col<utl::cstr, UTL_NAME("from")> from_;
  utl::csv_col<utl::cstr, UTL_NAME("to")> to_;
  utl::csv_col<std::time_t, UTL_NAME("enter")> enter_;
  utl::csv_col<std::time_t, UTL_NAME("exit")> exit_;
  utl::csv_col<utl::cstr, UTL_NAME("category")> category_;
  utl::csv_col<std::uint32_t, UTL_NAME("train_nr")> train_nr_;
  utl::csv_col<std::uint16_t, UTL_NAME("passengers")> passengers_;
};

std::uint32_t get_train_nr(light_connection const* lc, std::uint32_t expected) {
  auto ci = lc->full_con_->con_info_;
  auto const first_train_nr = ci->train_nr_;
  for (; ci != nullptr; ci = ci->merged_with_) {
    if (ci->train_nr_ == expected) {
      return expected;
    }
  }
  return first_train_nr;
}

trip* find_trip(schedule const& sched, std::uint32_t from_station_idx,
                std::uint32_t to_station_idx, time enter_time, time exit_time,
                std::uint32_t train_nr) {
  auto const from_station = sched.station_nodes_.at(from_station_idx).get();
  trip* trp_found = nullptr;
  from_station->for_each_route_node([&](node const* route_node) {
    for (auto const& e : route_node->edges_) {
      if (e.type() != ::motis::edge::ROUTE_EDGE) {
        continue;
      }
      auto const& conns = e.m_.route_edge_.conns_;
      for (auto lc = std::lower_bound(begin(conns), end(conns),
                                      light_connection{enter_time});
           lc != end(conns) && lc->d_time_ == enter_time; lc = std::next(lc)) {
        auto const current_train_nr = get_train_nr(lc, train_nr);
        for (auto trp : *sched.merged_trips_[lc->trips_]) {
          for (auto const& stop : access::stops(trp)) {
            if (stop.get_station_id() == to_station_idx && stop.has_arrival() &&
                stop.arr_lcon().a_time_ == exit_time) {
              if (current_train_nr == train_nr) {
                trp_found = trp;
                return;
              } else if (trp_found == nullptr) {
                trp_found = trp;
              }
            }
          }
        }
      }
    }
  });
  return trp_found;
}

void debug_trip_match(schedule const& sched, std::uint32_t from_station_idx,
                      std::uint32_t to_station_idx, time enter_time,
                      time exit_time, std::uint32_t train_nr,
                      std::string_view category, std::ofstream& match_log,
                      duration max_time_diff = 60) {
  auto const from_station = sched.station_nodes_.at(from_station_idx).get();
  auto const earliest_dep = enter_time > max_time_diff
                                ? static_cast<time>(enter_time - max_time_diff)
                                : static_cast<time>(0);
  auto const latest_dep = static_cast<time>(enter_time + max_time_diff);
  auto const expected_travel_time = static_cast<int>(exit_time - enter_time);
  match_log << "  stations: " << sched.stations_[from_station_idx]->name_
            << " => " << sched.stations_[to_station_idx]->name_ << "\n";
  match_log << "  possible matching trips with departure in ["
            << format_time(earliest_dep) << ", " << format_time(latest_dep)
            << "], expected travel_time=" << expected_travel_time << ":\n";
  from_station->for_each_route_node([&](node const* route_node) {
    for (auto const& e : route_node->edges_) {
      if (e.type() != ::motis::edge::ROUTE_EDGE) {
        continue;
      }
      auto const& conns = e.m_.route_edge_.conns_;
      for (auto lc = std::lower_bound(begin(conns), end(conns),
                                      light_connection{earliest_dep});
           lc != end(conns); lc = std::next(lc)) {
        if (lc->d_time_ > latest_dep) {
          break;
        }
        if (lc->valid_ == 0) {
          continue;
        }
        auto const current_train_nr = get_train_nr(lc, train_nr);
        auto found_one = false;

        for (auto trp : *sched.merged_trips_[lc->trips_]) {
          for (auto const& stop : access::stops(trp)) {
            if (stop.get_station_id() == to_station_idx && stop.has_arrival()) {
              auto const arrival_time = stop.arr_lcon().a_time_;
              auto const travel_time =
                  static_cast<int>(arrival_time) - lc->d_time_;
              if (travel_time < 0) {
                continue;
              }
              if (!found_one) {
                found_one = true;
                auto const& current_category =
                    sched.categories_[lc->full_con_->con_info_->family_]->name_;
                fmt::print(
                    match_log,
                    "    dep={} [{:+3}], train_nr={:6} [{}], "
                    "category={:6} [{}] =>",
                    format_time(lc->d_time_),
                    static_cast<int>(lc->d_time_) -
                        static_cast<int>(enter_time),
                    current_train_nr, current_train_nr == train_nr ? "✓" : "✗",
                    current_category, current_category == category ? "✓" : "✗");
              } else {
                match_log << " |";
              }
              fmt::print(
                  match_log,
                  " arr={} [{:+3}], trip_train_nr={:6} [{}], tt={:3} [{:+3}]",
                  format_time(arrival_time),
                  static_cast<int>(arrival_time) - static_cast<int>(exit_time),
                  trp->id_.primary_.get_train_nr(),
                  trp->id_.primary_.get_train_nr() == train_nr ? "✓" : "✗",
                  travel_time, travel_time - expected_travel_time);
            }
          }
        }
        if (found_one) {
          match_log << "\n";
        }
      }
    }
  });
  match_log << "\n";
}

std::optional<time> get_footpath_duration(schedule const& sched,
                                          std::uint32_t from_station_idx,
                                          std::uint32_t to_station_idx) {
  for (auto const& fp :
       sched.stations_[from_station_idx]->outgoing_footpaths_) {
    if (fp.to_station_ == to_station_idx) {
      return {fp.duration_};
    }
  }
  return {};
}

std::optional<transfer_info> get_transfer_info(
    schedule const& sched, compact_journey const& partial_journey,
    std::uint32_t enter_station_idx, time enter_time) {
  if (partial_journey.legs_.empty()) {
    return {};
  }
  auto const& prev_leg = partial_journey.legs_.back();
  if (prev_leg.exit_station_id_ == enter_station_idx) {
    auto const journey_ic =
        static_cast<duration>(enter_time - prev_leg.exit_time_);
    return transfer_info{
        std::min(static_cast<duration>(
                     sched.stations_[enter_station_idx]->transfer_time_),
                 journey_ic),
        transfer_info::type::SAME_STATION};
  } else {
    auto const walk_duration =
        get_footpath_duration(sched, prev_leg.exit_station_id_,
                              enter_station_idx)
            .value_or(enter_time - prev_leg.exit_time_);
    return transfer_info{static_cast<duration>(walk_duration),
                         transfer_info::type::FOOTPATH};
  }
}

std::size_t load_journeys(schedule const& sched, paxmon_data& data,
                          std::string const& journey_file,
                          std::string const& match_log_file) {
  std::size_t journey_count = 0;
  auto error_count = 0ULL;
  auto trip_not_found_count = 0ULL;
  auto station_not_found_count = 0ULL;
  auto invalid_timestamp_count = 0ULL;

  auto buf = utl::file(journey_file.data(), "r").content();
  auto const file_content = utl::cstr{buf.data(), buf.size()};

  std::ofstream match_log;
  if (!match_log_file.empty()) {
    match_log.open(match_log_file);
  }

  auto current_id = std::optional<std::pair<std::uint64_t, std::uint64_t>>{};
  auto current_journey = compact_journey{};
  std::uint16_t current_passengers = 0;
  auto current_invalid = false;

  auto const finish_journey = [&]() {
    if (current_id) {
      if (!current_invalid) {
        ++journey_count;
        auto const id =
            static_cast<std::uint64_t>(data.graph_.passenger_groups_.size());
        data.graph_.passenger_groups_.emplace_back(
            std::make_unique<passenger_group>(
                passenger_group{current_journey, current_passengers, id,
                                data_source{current_id.value().first,
                                            current_id.value().second}}));
      } else {
        ++error_count;
      }
    }
    current_journey = {};
    current_invalid = false;
  };

  utl::line_range<utl::buf_reader>{file_content}  //
      | utl::csv<row>()  //
      |
      utl::for_each([&](auto&& row) {
        auto const id = std::make_pair(row.id_.val(), row.secondary_id_.val());
        if (id != current_id) {
          finish_journey();
          current_id = id;
          current_passengers = row.passengers_.val();
        }
        if (row.leg_type_.val() == "FOOT") {
          return;
        }
        auto const from_station_idx =
            get_station_idx(sched, row.from_.val().view());
        auto const to_station_idx =
            get_station_idx(sched, row.to_.val().view());
        auto const enter_time =
            unix_to_motistime(sched.schedule_begin_, row.enter_.val());
        auto const exit_time =
            unix_to_motistime(sched.schedule_begin_, row.exit_.val());
        if (!from_station_idx || !to_station_idx) {
          current_invalid = true;
          ++station_not_found_count;
          if (!from_station_idx) {
            match_log << "[" << current_id
                      << "] Station not found: " << row.from_.val().view()
                      << "\n";
          }
          if (!to_station_idx) {
            match_log << "[" << current_id
                      << "] Station not found: " << row.to_.val().view()
                      << "\n";
          }
          return;
        }
        if (enter_time == INVALID_TIME || exit_time == INVALID_TIME) {
          current_invalid = true;
          ++invalid_timestamp_count;
          if (enter_time == INVALID_TIME) {
            match_log << "[" << current_id << "] Invalid enter timestamp: "
                      << format_unix_time(row.enter_.val()) << "\n";
          }
          if (exit_time == INVALID_TIME) {
            match_log << "[" << current_id << "] Invalid exit timestamp: "
                      << format_unix_time(row.exit_.val()) << "\n";
          }
          return;
        }
        auto const trp =
            find_trip(sched, from_station_idx.value(), to_station_idx.value(),
                      enter_time, exit_time, row.train_nr_.val());
        if (trp == nullptr) {
          current_invalid = true;
          ++trip_not_found_count;

          if (match_log) {
            fmt::print(match_log,
                       "[{}] Trip not found: from={:7}, to={:7}, enter={}, "
                       "exit={}, train_nr={:6}, category={:6}, leg={}\n",
                       current_id, row.from_.val().view(), row.to_.val().view(),
                       format_unix_time(row.enter_.val()),
                       format_unix_time(row.exit_.val()), row.train_nr_.val(),
                       row.category_.val().view(),
                       current_journey.legs_.size());
            debug_trip_match(sched, from_station_idx.value(),
                             to_station_idx.value(), enter_time, exit_time,
                             row.train_nr_.val(), row.category_.val().view(),
                             match_log);
          }
          return;
        }
        auto enter_transfer = get_transfer_info(
            sched, current_journey, from_station_idx.value(), enter_time);
        current_journey.legs_.emplace_back(journey_leg{
            to_extern_trip(sched, trp), from_station_idx.value(),
            to_station_idx.value(), enter_time, exit_time, enter_transfer});
      });

  finish_journey();

  if (error_count > 0) {
    LOG(warn) << "could not load " << error_count << " journeys";
    LOG(warn) << station_not_found_count << " stations not found";
    LOG(warn) << trip_not_found_count << " trips not found";
    LOG(warn) << invalid_timestamp_count << " invalid timestamps";
  }

  return journey_count;
}

}  // namespace motis::paxmon::loader::csv