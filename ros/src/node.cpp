// MIT License
//
// Copyright (c) 2025 Easton Potokar, Taylor Pool, and Michael Kaess
//
// Adapted from KISS-ICP ROS 2 node
// (Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss)
#include <memory>
#include <utility>

// FORM-ROS
#include "node.hpp"
#include "ros_pc2.h"

// FORM
#include "form/form.hpp"

// ROS 2 headers
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {
gtsam::Pose3 LookupTransform(const std::string &target_frame,
                             const std::string &source_frame,
                             const std::unique_ptr<tf2_ros::Buffer> &tf2_buffer) {
  std::string err_msg;
  if (tf2_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero,
                               &err_msg)) {
    try {
      auto tf = tf2_buffer->lookupTransform(target_frame, source_frame,
                                            tf2::TimePointZero);
      return tf2::transformToGtsam(tf);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "%s", ex.what());
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "Failed to find tf. Reason=%s",
              err_msg.c_str());
  return gtsam::Pose3::Identity();
}
} // namespace

namespace form_ros {

EstimatorNode::EstimatorNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("form_node", options) {
  // clang-format off

  // ROS parameters
  base_frame_       = declare_parameter<std::string>("base_frame", base_frame_);
  lidar_odom_frame_ = declare_parameter<std::string>("lidar_odom_frame", lidar_odom_frame_);
  publish_odom_tf_  = declare_parameter<bool>("publish_odom_tf", true);
  invert_odom_tf_   = declare_parameter<bool>("invert_odom_tf", true);
  publish_debug_clouds_   = declare_parameter<bool>("publish_debug_clouds", false);
  position_covariance_    = declare_parameter<double>("position_covariance", 0.1);
  orientation_covariance_ = declare_parameter<double>("orientation_covariance", 0.1);

  // LiDAR format
  std::string model_name = declare_parameter<std::string>("lidar_model", "");
  // If the name is given specifically, use it
  if (!model_name.empty()) {
    auto it = LIDAR_FORMATS.find(model_name);
    if (it == LIDAR_FORMATS.end()) {
      RCLCPP_WARN(this->get_logger(), "Unknown LiDAR model '%s', defaulting to inferring model", model_name.c_str());
    } else {
      lidar_format_ = it->second;
    }
  }

  // FORM parameters
  form::Estimator::Params params;

  // Feature extraction
  auto min_range = declare_parameter<double>("min_range", 1.0);
  auto max_range = declare_parameter<double>("max_range", 100.0);
  params.extraction.min_norm_squared = min_range * min_range;
  params.extraction.max_norm_squared = max_range * max_range;
  if(lidar_format_.has_value()) {
    params.extraction.num_rows = lidar_format_->num_rows;
    params.extraction.num_columns = lidar_format_->num_columns;
  } else {
    params.extraction.num_rows = declare_parameter<int>("num_rows", 0);
    params.extraction.num_columns = declare_parameter<int>("num_columns", 0);
  }
  params.extraction.neighbor_points         = declare_parameter<int>("neighbor_points", params.extraction.neighbor_points);
  params.extraction.num_sectors             = declare_parameter<int>("num_sectors", params.extraction.num_sectors);
  params.extraction.planar_threshold        = declare_parameter<double>("planar_threshold", params.extraction.planar_threshold);
  params.extraction.planar_feats_per_sector = declare_parameter<int>("planar_feats_per_sector", params.extraction.planar_feats_per_sector);
  params.extraction.point_feats_per_sector  = declare_parameter<int>("point_feats_per_sector", params.extraction.point_feats_per_sector);
  params.extraction.radius                  = declare_parameter<double>("radius", params.extraction.radius);
  params.extraction.min_points              = declare_parameter<int>("min_points", params.extraction.min_points);

  // Optimization
  params.matcher.max_dist_matching  = declare_parameter<double>("max_dist_matching", params.matcher.max_dist_matching);
  params.matcher.new_pose_threshold = declare_parameter<double>("new_pose_threshold", params.matcher.new_pose_threshold);
  params.matcher.max_num_rematches  = declare_parameter<int>("max_num_rematches", params.matcher.max_num_rematches);

  // Constraints
  params.constraints.disable_smoothing =
      declare_parameter<bool>("disable_smoothing", params.constraints.disable_smoothing);

  // Mapping
  params.scans.max_num_keyscans         = declare_parameter<int>("max_num_keyscans", params.scans.max_num_keyscans);
  params.scans.max_num_recent_scans     = declare_parameter<int>("max_num_recent_scans", params.scans.max_num_recent_scans);
  params.scans.max_steps_unused_keyscan = declare_parameter<int>("max_steps_unused_keyscan", params.scans.max_steps_unused_keyscan);
  params.scans.keyscan_match_ratio      = declare_parameter<double>("keyscan_match_ratio", params.scans.keyscan_match_ratio);
  params.map.min_dist_map               = declare_parameter<double>("min_dist_map", params.map.min_dist_map);

  // Misc
  params.num_threads = declare_parameter<int>("num_threads", params.num_threads);

  // Construct the FORM estimator
  estimator_ = form::Estimator(params);

  // Initialize subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_topic", rclcpp::SensorDataQoS(),
      std::bind(&EstimatorNode::register_frame, this, std::placeholders::_1));

  // Initialize publishers
  rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("form/odometry", qos);
  if (publish_debug_clouds_) {
    planar_publisher_   = create_publisher<sensor_msgs::msg::PointCloud2>("form/kp_planar", qos);
    point_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("form/kp_point", qos);

    map_planar_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("form/map_planar", qos);
    map_point_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("form/map_point", qos);
  }

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_buffer_->setUsingDedicatedThread(true);
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

  RCLCPP_INFO(this->get_logger(), "FORM ROS 2 odometry node initialized");
  // clang-format on
}

void EstimatorNode::register_frame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  // Convert PointCloud2 -> RawPoints
  auto raw_points = form_ros::load_pc2(msg);

  // Infer LiDAR sizes if not set by user
  if (estimator_.m_extractor.params.num_rows == 0 ||
      estimator_.m_extractor.params.num_columns == 0) {
    const auto [num_rows, num_columns] = form_ros::infer_lidar_size(raw_points);
    if (estimator_.m_extractor.params.num_rows == 0) {
      estimator_.m_extractor.params.num_rows = num_rows;
    }
    if (estimator_.m_extractor.params.num_columns == 0) {
      estimator_.m_extractor.params.num_columns = num_columns;
    }
    RCLCPP_INFO(this->get_logger(),
                "Inferred LiDAR sizes: num_rows=%lu, num_columns=%lu", num_rows,
                num_columns);
  }
  // Infer lidar ordering properties if not set by user
  if (!lidar_format_.has_value()) {
    lidar_format_ = form_ros::infer_lidar_order(
        raw_points, estimator_.m_extractor.params.num_rows,
        estimator_.m_extractor.params.num_columns);
    RCLCPP_INFO(this->get_logger(),
                "Inferred LiDAR ordering: %s, %s, sequential firing order",
                lidar_format_->row_major ? "row major" : "column major",
                lidar_format_->all_points_present ? "all points present"
                                                  : "not all points present");
  }

  // RawPoints -> Structured form::PointXYZf
  const auto points =
      form_ros::reorder(raw_points, *lidar_format_, this->get_logger());

  // Register frame with FORM
  const auto &[planar_kp, point_kp] = estimator_.register_scan(points);

  // Get the current pose estimate
  const gtsam::Pose3 pose = estimator_.current_lidar_estimate();

  // Publish odometry
  publish_odometry(pose, msg->header);

  // Publish debug clouds if requested
  if (publish_debug_clouds_) {
    // current keypoints
    publish_clouds(planar_kp, planar_publisher_, msg->header);
    publish_clouds(point_kp, point_publisher_, msg->header);
    // current map
    const auto [map_planar, map_point] = estimator_.current_map();
    auto local_map_header = msg->header;
    local_map_header.frame_id = lidar_odom_frame_;
    publish_clouds(map_planar, map_planar_publisher_, local_map_header);
    publish_clouds(map_point, map_point_publisher_, local_map_header);
  }
}

void EstimatorNode::publish_odometry(const gtsam::Pose3 &form_pose,
                                     const std_msgs::msg::Header &header) {
  // If necessary, transform the ego-centric pose to the specified
  // base_link/base_footprint frame
  const auto cloud_frame_id = utils::FixFrameId(header.frame_id);
  const auto egocentric_estimation =
      (base_frame_.empty() || base_frame_ == cloud_frame_id);
  const auto moving_frame = egocentric_estimation ? cloud_frame_id : base_frame_;
  const auto pose = [&]() -> gtsam::Pose3 {
    if (egocentric_estimation)
      return form_pose;
    const gtsam::Pose3 cloud2base =
        LookupTransform(base_frame_, cloud_frame_id, tf2_buffer_);
    return cloud2base * form_pose * cloud2base.inverse();
  }();

  // Broadcast the tf
  if (publish_odom_tf_) {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = header.stamp;
    if (invert_odom_tf_) {
      transform_msg.header.frame_id = moving_frame;
      transform_msg.child_frame_id = lidar_odom_frame_;
      transform_msg.transform = tf2::gtsamToTransform(pose.inverse());
    } else {
      transform_msg.header.frame_id = lidar_odom_frame_;
      transform_msg.child_frame_id = moving_frame;
      transform_msg.transform = tf2::gtsamToTransform(pose);
    }
    tf_broadcaster_->sendTransform(transform_msg);
  }

  // Publish odometry msg
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = header.stamp;
  odom_msg.header.frame_id = lidar_odom_frame_;
  odom_msg.child_frame_id = moving_frame;
  odom_msg.pose.pose = tf2::gtsamToPose(pose);
  odom_msg.pose.covariance.fill(0.0);
  odom_msg.pose.covariance[0] = position_covariance_;
  odom_msg.pose.covariance[7] = position_covariance_;
  odom_msg.pose.covariance[14] = position_covariance_;
  odom_msg.pose.covariance[21] = orientation_covariance_;
  odom_msg.pose.covariance[28] = orientation_covariance_;
  odom_msg.pose.covariance[35] = orientation_covariance_;
  odom_publisher_->publish(std::move(odom_msg));
}

} // namespace form_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(form_ros::EstimatorNode)
