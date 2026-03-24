#pragma once

#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

struct LidarFormat {
  int num_rows = 0;
  int num_columns = 0;
  bool row_major = false;
  bool all_points_present = false;
  std::optional<std::vector<long>> map_row_to_fire = std::nullopt;
  double min_range = 0.0;
  double max_range = 0.0;
};

inline std::ostream &operator<<(std::ostream &os, const LidarFormat &format) {
  os << "LidarFormat(rows=" << format.num_rows << ", cols=" << format.num_columns
     << ", row_major=" << format.row_major
     << ", all_points=" << format.all_points_present
     << ", min_range=" << format.min_range << ", max_range=" << format.max_range
     << " )";
  return os;
}

/// Invert a map from firing order -> row index into a map from row index -> firing
/// order
///
/// By default printing out the row index in the order they are fired gives
/// map_fire_to_row, but for some reordering logic it's more convenient to have the
/// inverse
inline std::vector<long> invert_map(const std::vector<long> &map_fire_to_row) {
  std::vector<long> map_row_to_fire(map_fire_to_row.size());
  for (size_t i = 0; i < map_fire_to_row.size(); ++i) {
    map_row_to_fire[map_fire_to_row[i]] = i;
  }
  return map_row_to_fire;
}

inline std::vector<long> default_row_to_fire(size_t num_rows) {
  std::vector<long> map_row_to_fire(num_rows);
  for (size_t i = 0; i < num_rows; ++i) {
    map_row_to_fire[i] = i;
  }
  return map_row_to_fire;
}

inline const std::map<std::string, LidarFormat> LIDAR_FORMATS = {
    {"Hesai-QT64", {64, 1200, false, false, default_row_to_fire(64), 1.0, 60.0}},
    {"VLP-16", {16, 3624, false, true, default_row_to_fire(16), 0.1, 100.0}},
    // Add more models as needed
};