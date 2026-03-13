# FORM ROS 2 Wrapper

This ROS 2 node is based on the [KISS-ICP](https://github.com/PRBonn/kiss-icp) ROS 2 wrapper.
Thanks to the KISS-ICP developers for their clean and well-structured ROS node implementation.

### How to build

You will need the same dependencies as FORM (most of which are available as ROS packages),
- [eigen3](https://libeigen.gitlab.io/eigen/docs-nightly/)
- [gtsam](https://github.com/borglab/gtsam/)
- [tbb](https://github.com/uxlfoundation/oneTBB)
- [tsl::robin_map](https://github.com/Tessil/robin-map) (will be pulled from source if not found)

Once those are installed, simply clone FORM to your workspace and build,

```sh
git clone https://github.com/contagon/form
colcon build
source ./install/setup.bash
```

### How to run

The main launch file is `odometry.launch.py` which will launch the odometry node. FORM has three required inputs, **topic name**, **number of scanlines/rings/columns**, and **number of rows/circular count**. This last two are required for FORM's feature extraction.

To view all the available arguments, you can run:

```sh
ros2 launch form odometry.launch.py --show-args
```
which will output the following:

| Parameter          | Default      | Description                          |
|--------------------|--------------|--------------------------------------|
| `topic`            | `None`       | Input point cloud topic              |
| `num_columns`      | `None`       | LiDAR image width (columns)          |
| `num_rows`         | `None`       | LiDAR image height (rows)            |
| `min_range`        | `1.0`        | Minimum LiDAR range                  |
| `max_range`        | `100.0`      | Maximum LiDAR range                  |
| `visualize`        | `true`       | Launch RViz and publish point clouds |
| `bagfile`          | `''`         | Optional rosbag file/folder to play  |
| `base_frame`       | `''`         | Base frame id                        |
| `lidar_odom_frame` | `odom_lidar` | Odometry frame id                    |
| `publish_odom_tf`  | `True`       | Publish odom->base TF                |
| `invert_odom_tf`   | `True`       | Invert published odom TF             |
| `use_sim_time`     | `True`       | Use simulation time                  |

If a bagfile is provided, the node will play the bagfile and process the point clouds. If not, it will just subscribe to the topic and process incoming point clouds in real-time.

Thus as an example

```sh
ros2 launch form odometry.launch.py bagfile:=<path_to_rosbag> topic:=<topic_name> num_columns:=<num_columns> num_rows:=<num_rows>
```

### Pointcloud Format

FORM requires point clouds to be in row-major order with no dropped points for its feature extraction method. It does it's best to infer the ordering and density of the point cloud using the following heuristics,

| Format         | Heuristic to Infer                                       |
|----------------|----------------------------------------------------------|
| all points     | Point cloud size equaling `num_columns` * `num_rows`     |
| dropped points | Point cloud size not equaling `num_columns` * `num_rows` |
| row-major      | Stationary ring number for first few points              |
| column-major   | Increasing ring number for first few points              |

IMPORTANT: If you're point cloud *does not* is not in row or column major format, FORM will crash! Please open an issue and we can add a flag to handle you're appropriate cloud formatting. (For example, I've seen some velodyne point clouds have ring order returned as 0, 8, 1, 9, 2, 10, ... This will break things)

NOTE: If your point cloud is row major and has dropped invalid points, there is usually no way to figure out where along the scan line the dropped points belong. FORM places all of them at the end. This may have an impact on feature extraction, but things generally *should* still work.


### Development

We're not huge fans of installing ROS2 system wide, so this folder is setup to install all dependencies using [pixi](https://pixi.prefix.dev/latest/) and [robostack](https://robostack.github.io/index.html).

Install pixi, then building is all done in a single command
```sh
pixi run build
source install/setup.sh
```

If you need to utilize ROS2 commands, you can do either of the following,
```sh
pixi run ros2 <your command>
# enter into a pixi shell
pixi shell
ros2 <your command>
```

If you have the oxford spires dataset installed using [evalio](https://github.com/contagon/evalio/tree/master), you can use the following commands to launch and run a trajectory from it,
```sh
pixi run oxford
``