# FORM ROS 2 Wrapper

This ROS 2 node is based on the [KISS-ICP](https://github.com/PRBonn/kiss-icp) ROS 2 wrapper.
Thanks to the KISS-ICP developers for their clean and well-structured ROS node implementation.

### How to build

You should not need any extra dependency, just clone and build:

```sh
git clone https://github.com/contagon/form
colcon build
source ./install/setup.bash
```

### How to run

The only required argument to provide is the **topic name** so FORM knows which PointCloud2 to process:

```sh
ros2 launch form odometry.launch.py bagfile:=<path_to_rosbag> topic:=<topic_name>
```

You can optionally launch the node with any bagfile, and play the bagfiles on a different shell:

```sh
ros2 launch form odometry.launch.py topic:=<topic_name>
```

and then,

```sh
ros2 bag play <path>*.bag
```