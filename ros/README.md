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

## Out of source builds

Good news! If you don't have git or you don't need to change the core FORM library, you can just
copy paste this folder into your ROS2 workspace and build as usual. The build system will fetch
the latest stable release for you.

## Looking how to run KITTI on ROS along with FORM?

I believe you could use the python API instead, to avoid converting all the data to a format that is
not the original one. If you still insist in doing it, I've created a separate repository for this,
just clone and build [this package](https://github.com/nachovizzo/kiss_icp_kitti).
