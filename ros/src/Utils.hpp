// MIT License
//
// Copyright (c) 2025 Easton Potokar, Taylor Pool, and Michael Kaess
//
// Adapted from KISS-ICP ROS 2 utility code
// (Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss)
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <regex>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>

// ROS 2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "form/feature/features.hpp"
#include "form/utils.hpp"

namespace tf2 {

inline geometry_msgs::msg::Transform gtsamToTransform(const gtsam::Pose3 &T) {
  geometry_msgs::msg::Transform t;
  t.translation.x = T.translation().x();
  t.translation.y = T.translation().y();
  t.translation.z = T.translation().z();

  auto q = T.rotation().toQuaternion();
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();

  return t;
}

inline geometry_msgs::msg::Pose gtsamToPose(const gtsam::Pose3 &T) {
  geometry_msgs::msg::Pose t;
  t.position.x = T.translation().x();
  t.position.y = T.translation().y();
  t.position.z = T.translation().z();

  auto q = T.rotation().toQuaternion();
  t.orientation.x = q.x();
  t.orientation.y = q.y();
  t.orientation.z = q.z();
  t.orientation.w = q.w();

  return t;
}

inline gtsam::Pose3
transformToGtsam(const geometry_msgs::msg::TransformStamped &transform) {
  const auto &t = transform.transform;
  return gtsam::Pose3(
      gtsam::Rot3::Quaternion(t.rotation.w, t.rotation.x, t.rotation.y,
                              t.rotation.z),
      gtsam::Point3(t.translation.x, t.translation.y, t.translation.z));
}
} // namespace tf2

namespace form_ros::utils {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;

inline std::string FixFrameId(const std::string &frame_id) {
  return std::regex_replace(frame_id, std::regex("^/"), "");
}

inline std::unique_ptr<PointCloud2> CreatePointCloud2Msg(const size_t n_points,
                                                         const Header &header) {
  auto cloud_msg = std::make_unique<PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
  cloud_msg->header = header;
  cloud_msg->header.frame_id = FixFrameId(cloud_msg->header.frame_id);
  cloud_msg->fields.clear();
  int offset = 0;
  offset = addPointField(*cloud_msg, "x", 1, PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "y", 1, PointField::FLOAT32, offset);
  offset = addPointField(*cloud_msg, "z", 1, PointField::FLOAT32, offset);
  offset += sizeOfPointField(PointField::FLOAT32);

  // Resize the point cloud accordingly
  cloud_msg->point_step = offset;
  cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
  cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
  modifier.resize(n_points);
  return cloud_msg;
}

template <typename PointT>
inline void FillPointCloud2XYZ(const std::vector<PointT> &points, PointCloud2 &msg) {
  sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
  for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
    const auto &point = points[i];
    *msg_x = static_cast<float>(point.x);
    *msg_y = static_cast<float>(point.y);
    *msg_z = static_cast<float>(point.z);
  }
}

template <typename PointT>
inline std::unique_ptr<PointCloud2>
FeatsToPointCloud2(const std::vector<PointT> &points, const Header &header) {
  auto msg = CreatePointCloud2Msg(points.size(), header);
  FillPointCloud2XYZ(points, *msg);
  return msg;
}

} // namespace form_ros::utils
