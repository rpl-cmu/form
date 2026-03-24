# FORM ROS 2 Wrapper

This ROS 2 node is based on the [KISS-ICP](https://github.com/PRBonn/kiss-icp) ROS 2 wrapper.
Thanks to the KISS-ICP developers for their clean and well-structured ROS node implementation.

### Building

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

### Running

The main launch file is `odometry.launch.py` which will launch the odometry node. FORM only requires the point cloud **topic** to operate.
To view all the available arguments, you can run:

```sh
ros2 launch form odometry.launch.py --show-args
```
which will output the following:

| Parameter          | Default       | Description                                                                                                                                                            |
|--------------------|---------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `topic`            | `None`        | Input point cloud topic (**Only required parameter!**)                                                                                                                 |
| `lidar_model`      | `"" (auto)`   | LiDAR model parameters to use. Empty string lets the node infer/use model defaults from the data. See `format.hpp` for options. Sets num_columns, num_rows, ranges, etc. |
| `num_columns`      | `Inferred`    | LiDAR image width (columns). Overrides value from lidar_model.                                                                                                         |
| `num_rows`         | `Inferred`    | LiDAR image height (rows). Overrides value from lidar_model.                                                                                                           |
| `min_range`        | `0.0 (auto)`  | Minimum LiDAR range. `0.0` means infer/use defaults from lidar_model (falling back to the sensor’s nominal minimum range, currently 0.1 m).                            |
| `max_range`        | `0.0 (auto)`  | Maximum LiDAR range. `0.0` means infer/use defaults from lidar_model (falling back to the sensor’s nominal maximum range, currently 100.0 m).                          |
| `visualize`        | `true`        | Launch RViz and publish point clouds                                                                                                                                    |
| `bagfile`          | `''`         | Optional rosbag file/folder to play                                                                                                      |
| `base_frame`       | `''`         | Base frame id                                                                                                                            |
| `lidar_odom_frame` | `odom_lidar` | Odometry frame id                                                                                                                        |
| `publish_odom_tf`  | `True`       | Publish odom->base TF                                                                                                                    |
| `invert_odom_tf`   | `True`       | Invert published odom TF                                                                                                                 |
| `use_sim_time`     | `True`       | Use simulation time                                                                                                                      |

If a bagfile is provided, the node will play the bagfile and process the point clouds. If not, it will just subscribe to the topic and process incoming point clouds in real-time.

Thus as an example

```sh
ros2 launch form odometry.launch.py bagfile:=<path_to_rosbag> topic:=<topic_name>
```

### Pointcloud Format

FORM requires point clouds to be in row-major order with no dropped points for its feature extraction method. It does it's best to infer the size, ordering, and density of the point cloud to reorder things properly. There are a handful of cases that can prove suboptimal if not manually set however,

1. **Column Major, Dropped Invalid Points**: In this case it's difficult (but not impossible) to tell where along the scanline the dropped/invalid points come from. To be able to do so, we need the returned "firing order" of a column. This is often sequential, but I've seen ring orders such as 0, 8, 1, 9, ... so forth. To set this, add your LiDAR format to format.hpp and pass it in with `lidar_model`. If it is not set, all dropped points will be added at the end of the scanline. FORM will still run fine, but likely not quite as accurately.
2. **Row Major, Dropped Invalid Points**: This is very rare, but when it does occur it *is* impossible to tell where along the scanline points belong. In this case, FORM places them at the end of the scanline. 

FORM will output details about it's point cloud format inference for debugging purposes if you ever hit any edge cases that don't seem to be working. Generally fixes *should* be as easy as setting the parameters that FORM incorrectly inferred.

### Development

We're not huge fans of installing ROS2 system wide, so this folder is setup to install all dependencies using [pixi](https://pixi.prefix.dev/latest/) and [robostack](https://robostack.github.io/index.html).

Install pixi, then building is all done in a single command from the `ros/` directory
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
```