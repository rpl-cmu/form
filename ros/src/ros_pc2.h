// Adapted from evalio's pc2_conversions.h
// https://github.com/contagon/evalio/blob/main/cpp/bindings/ros_pc2.h
//
// Converts ROS PointCloud2 messages into organized form::PointXYZf vectors
// (row-major, num_rows * num_columns) suitable for FORM's feature extraction.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <form/utils.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace form_ros {

// ----------------------------- Data getters ----------------------------- //
// Build a lambda that reads a field of type T from raw PointCloud2 byte data
// at the given offset, handling the various PointField datatypes.
template <typename T>
std::function<void(T &, const uint8_t *)> data_getter(uint8_t datatype,
                                                      uint32_t offset) {
  using sensor_msgs::msg::PointField;
  switch (datatype) {
  case PointField::UINT8:
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(data[offset]);
    };
  case PointField::INT8:
    return [offset](T &value, const uint8_t *data) noexcept {
      value = static_cast<T>(*reinterpret_cast<const int8_t *>(data + offset));
    };
  case PointField::UINT16:
    return [offset](T &value, const uint8_t *data) noexcept {
      uint16_t v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::INT16:
    return [offset](T &value, const uint8_t *data) noexcept {
      int16_t v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::UINT32:
    return [offset](T &value, const uint8_t *data) noexcept {
      uint32_t v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::INT32:
    return [offset](T &value, const uint8_t *data) noexcept {
      int32_t v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::FLOAT32:
    return [offset](T &value, const uint8_t *data) noexcept {
      float v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  case PointField::FLOAT64:
    return [offset](T &value, const uint8_t *data) noexcept {
      double v;
      std::memcpy(&v, data + offset, sizeof(v));
      value = static_cast<T>(v);
    };
  default:
    throw std::runtime_error("Unsupported PointField datatype: " +
                             std::to_string(datatype));
  }
}

template <typename T> std::function<void(T &, const uint8_t *)> blank() {
  return [](T &, const uint8_t *) noexcept {};
}

// ------------------------- Reordering Helpers ------------------------- //
// Holds parsed per-point data before reordering into the organized grid.
struct RawPoint {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  uint8_t row = 0;
  uint16_t col = 0;
};

// Iterates through points to fill in columns
inline void
_fill_col(std::vector<RawPoint> &mm,
          std::function<void(uint16_t &col, const uint16_t &prev_col,
                             const uint8_t &prev_row, const uint8_t &curr_row)>
              func_col) {
  // fill out the first one to kickstart
  uint16_t prev_col = 0;
  uint8_t prev_row = mm[0].row;
  for (auto p = mm.begin() + 1; p != mm.end(); ++p) {
    func_col(p->col, prev_col, prev_row, p->row);
    prev_col = p->col;
    prev_row = p->row;
  }
}

// Fills in column index for row major order
inline void fill_col_row_major(std::vector<RawPoint> &mm) {
  auto func_col = [](uint16_t &col, const uint16_t &prev_col,
                     const uint8_t &prev_row, const uint8_t &curr_row) {
    if (prev_row != curr_row) {
      col = 0;
    } else {
      col = prev_col + 1;
    }
  };

  _fill_col(mm, func_col);
}

// Fills in column index for column major order
inline void fill_col_col_major(std::vector<RawPoint> &mm) {
  auto func_col = [](uint16_t &col, const uint16_t &prev_col,
                     const uint8_t &prev_row, const uint8_t &curr_row) {
    if (curr_row < prev_row) {
      col = prev_col + 1;
    } else {
      col = prev_col;
    }
  };

  _fill_col(mm, func_col);
}

inline std::vector<form::PointXYZf>
reorder_points(std::vector<RawPoint> &mm, size_t num_rows, size_t num_cols) {
  std::vector<form::PointXYZf> output(num_rows * num_cols,
                                      form::PointXYZf(0.0f, 0.0f, 0.0f));

  for (auto p : mm) {
    if (p.row >= num_rows || p.col >= num_cols) {
      std::cerr << "Warning: Point with row " << static_cast<int>(p.row)
                << " and col " << p.col
                << " is out of bounds for num_rows=" << num_rows
                << " and num_cols=" << num_cols << ". Skipping this point."
                << std::endl;
      continue;
    }
    output[p.row * num_cols + p.col] = form::PointXYZf(p.x, p.y, p.z);
  }
  return output;
}

// ------------------------- Main Conversion ------------------------- //
inline std::vector<form::PointXYZf>
PointCloud2ToForm(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
                  int num_rows, int num_columns) {
  if (msg->is_bigendian) {
    throw std::runtime_error("Big-endian PointCloud2 is not supported");
  }

  const int n_points = static_cast<int>(msg->width) * static_cast<int>(msg->height);

  // Build field accessors
  std::function func_x = blank<float>();
  std::function func_y = blank<float>();
  std::function func_z = blank<float>();
  std::function func_row = blank<uint8_t>();

  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_row = false;

  for (const auto &field : msg->fields) {
    if (field.name == "x") {
      func_x = data_getter<float>(field.datatype, field.offset);
      has_x = true;
    } else if (field.name == "y") {
      func_y = data_getter<float>(field.datatype, field.offset);
      has_y = true;
    } else if (field.name == "z") {
      func_z = data_getter<float>(field.datatype, field.offset);
      has_z = true;
    } else if (field.name == "ring" || field.name == "row" || field.name == "channel") {
      func_row = data_getter<uint8_t>(field.datatype, field.offset);
      has_row = true;
    }
  }

  // Validate that all required fields were found
  if (!has_x || !has_y || !has_z) {
    throw std::runtime_error(
        "PointCloud2ToForm: Missing required point fields; expected fields 'x', 'y', and 'z'.");
  }
  if (!has_row) {
    throw std::runtime_error(
        "PointCloud2ToForm: Missing required row indicator field; expected one of 'ring', 'row', or 'channel'.");
  }

  // Parse all points
  std::vector<RawPoint> raw(n_points);
  const uint8_t *data = msg->data.data();
  for (size_t i = 0; i < n_points; ++i) {
    const uint8_t *pt = data + i * msg->point_step;
    func_x(raw[i].x, pt);
    func_y(raw[i].y, pt);
    func_z(raw[i].z, pt);
    func_row(raw[i].row, pt);
  }

  // Catch some potential edge cases with bad values
  if (n_points > num_columns * num_rows) {
    std::cerr << "Warning: PointCloud2 has more points than expected from num_rows "
                 "and num_columns. Please check your parameters."
              << std::endl;
  }

  // Infer properties of the cloud
  bool all_points_present = (n_points == num_rows * num_columns);
  bool row_major = true;
  if (n_points >= 2) {
    row_major = (raw[0].row == raw[1].row);
  }

  // Fill out column indices based on inferred ordering
  if (row_major) {
    fill_col_row_major(raw);
  } else {
    fill_col_col_major(raw);
  }

  return reorder_points(raw, num_rows, num_columns);
}

} // namespace form_ros
