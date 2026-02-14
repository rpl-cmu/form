# MIT License
#
# Copyright (c) 2025 Easton Potokar, Taylor Pool, and Michael Kaess
#
# Adapted from KISS-ICP launch file
# (Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss)
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class config:
    # LiDAR geometry
    num_rows: int = 64
    num_columns: int = 1024
    min_range: float = 1.0
    max_range: float = 100.0

    # Feature extraction
    neighbor_points: int = 5
    num_sectors: int = 6
    planar_threshold: float = 1.0
    planar_feats_per_sector: int = 50
    point_feats_per_sector: int = 3
    radius: float = 1.0
    min_points: int = 5

    # Optimization
    max_dist_matching: float = 0.8
    new_pose_threshold: float = 0.0001
    max_num_rematches: int = 30
    disable_smoothing: bool = False

    # Mapping
    max_num_keyscans: int = 50
    max_num_recent_scans: int = 10
    max_steps_unused_keyscan: int = 10
    keyscan_match_ratio: float = 0.1
    max_dist_map: float = 0.1

    # Misc
    num_threads: int = 0

    # Covariance diagonal values
    position_covariance: float = 0.1
    orientation_covariance: float = 0.1


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # ROS configuration
    pointcloud_topic = LaunchConfiguration("topic")
    visualize = LaunchConfiguration("visualize", default="true")

    # Optional ros bag play
    bagfile = LaunchConfiguration("bagfile", default="")

    # tf tree configuration
    base_frame = LaunchConfiguration("base_frame", default="")
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame", default="odom_lidar")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf", default=True)
    invert_odom_tf = LaunchConfiguration("invert_odom_tf", default=True)

    # FORM node
    form_node = Node(
        package="form",
        executable="form_node",
        name="form_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
        parameters=[
            {
                # ROS node configuration
                "base_frame": base_frame,
                "lidar_odom_frame": lidar_odom_frame,
                "publish_odom_tf": publish_odom_tf,
                "invert_odom_tf": invert_odom_tf,
                # LiDAR geometry
                "num_rows": config.num_rows,
                "num_columns": config.num_columns,
                "min_range": config.min_range,
                "max_range": config.max_range,
                # Feature extraction
                "neighbor_points": config.neighbor_points,
                "num_sectors": config.num_sectors,
                "planar_threshold": config.planar_threshold,
                "planar_feats_per_sector": config.planar_feats_per_sector,
                "point_feats_per_sector": config.point_feats_per_sector,
                "radius": config.radius,
                "min_points": config.min_points,
                # Optimization
                "max_dist_matching": config.max_dist_matching,
                "new_pose_threshold": config.new_pose_threshold,
                "max_num_rematches": config.max_num_rematches,
                "disable_smoothing": config.disable_smoothing,
                # Mapping
                "max_num_keyscans": config.max_num_keyscans,
                "max_num_recent_scans": config.max_num_recent_scans,
                "max_steps_unused_keyscan": config.max_steps_unused_keyscan,
                "keyscan_match_ratio": config.keyscan_match_ratio,
                "max_dist_map": config.max_dist_map,
                # Misc
                "num_threads": config.num_threads,
                # Fixed covariances
                "position_covariance": config.position_covariance,
                "orientation_covariance": config.orientation_covariance,
                # ROS CLI arguments
                "publish_debug_clouds": visualize,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("form"), "rviz", "form.rviz"]),
        ],
        condition=IfCondition(visualize),
    )

    bagfile_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", "--rate", "1", bagfile, "--clock", "1000.0"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", bagfile, "' != ''"])),
    )

    return LaunchDescription(
        [
            form_node,
            rviz_node,
            bagfile_play,
        ]
    )
