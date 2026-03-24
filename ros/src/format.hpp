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
};

inline std::ostream &operator<<(std::ostream &os, const LidarFormat &format) {
  os << "LidarFormat(rows=" << format.num_rows << ", cols=" << format.num_columns
     << ", row_major=" << format.row_major
     << ", all_points=" << format.all_points_present << ")";
  return os;
}

/// Invert a map from firing order to row index into a map from row index to firing
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
    {"VLP-16",
     {64, 1024, false, false,
      invert_map({0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15})}},

    // Add more models as needed
};