// MIT License
//
// Copyright (c) 2025 Easton Potokar, Taylor Pool, and Michael Kaess
//
// Adapted from KISS-ICP ROS 2 node
// (Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss)
#pragma once

// FORM
#include "form/form.hpp"

// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

namespace form_ros {

class OdometryServer : public rclcpp::Node {
public:
  /// OdometryServer constructor
  OdometryServer() = delete;
  explicit OdometryServer(const rclcpp::NodeOptions &options);

private:
  /// Register new frame
  void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

  /// Stream the estimated pose to ROS
  void PublishOdometry(const gtsam::Pose3 &pose,
                       const std_msgs::msg::Header &header);

  /// Stream the debugging point clouds for visualization
  void PublishClouds(const std::vector<form::PlanarFeat> &planar_keypoints,
                     const std::vector<form::PointFeat> &point_keypoints,
                     const std_msgs::msg::Header &header);

private:
  /// Tools for broadcasting TFs.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  bool invert_odom_tf_;
  bool publish_odom_tf_;
  bool publish_debug_clouds_;

  /// Data subscribers.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  /// Data publishers.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kpoints_publisher_;

  /// FORM estimator
  form::Estimator estimator_;

  /// Global/map coordinate frame.
  std::string lidar_odom_frame_{"odom_lidar"};
  std::string base_frame_{};

  /// Covariance diagonal
  double position_covariance_;
  double orientation_covariance_;
};

} // namespace form_ros
