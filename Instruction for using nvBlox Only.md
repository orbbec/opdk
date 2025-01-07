# OPDK

If you purchased OPDK from Orbbec, the software are all preloaded. You can jump to Getting Start section.

## Table of Contents

- [OPDK](#opdk)
  - [Table of Contents](#table-of-contents)
  - [Installation Instructions](#installation-instructions)
  - [Build Workspace](#build-workspace)
  - [Getting start](#getting-start)

## Installation Instructions

Get source code

```bash
cd /home/orbbec/Documents/orbbec
git clone git@github.com:orbbec/opdk.git
git submodule update --init --recursive
```

Install deb dependencies

```bash
sudo apt-get install git-lfs
git lfs install --skip-repo

sudo apt install vpi3-dev libnvvpi3 ros-humble-isaac-ros-nitros \
ros-humble-isaac-ros-managed-nitros ros-humble-isaac-ros-nitros-image-type \
ros-humble-isaac-ros-nitros-camera-info-type -y
sudo apt install libbenchmark-dev ros-humble-foxglove-bridge ros-humble-nav2-costmap-2d libgoogle-glog-dev -y

sudo apt install libgflags-dev nlohmann-json3-dev \
ros-humble-image-transport ros-humble-image-publisher ros-humble-camera-info-manager \
ros-humble-diagnostic-updater ros-humble-diagnostic-msgs ros-humble-statistics-msgs \
ros-humble-backward-ros libdw-dev ros-humble-image-transport \
ros-humble-image-transport-plugins ros-humble-compressed-image-transport \
ros-humble-rqt-tf-tree -y
sudo apt install ros-humble-rqt-image-view ros-humble-rviz2 clang-format -y
```

Install udev rules.

```bash
cd /home/orbbec/Documents/orbbec/opdk/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Build Workspace

```bash
cd /home/orbbec/Documents/orbbec/opdk
colcon build --packages-skip nvblox_test_data nvblox_test --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

## Getting start

Before getting start, please confirm all camera are connected on the correct usb port. The port numbers are: 2-1, 2-2, 2-3.1 and 2-3.3. The front camera should be connected to the 2-1 usb port as the primary camera. If you have different connection stradegy, please make coresponding modification in multi_camera_synced.launch.py.

When cuvSlam is not running, Orbbec camera does not need to output IR stream. Cameras can have more resolution options, i.e. 1280*800@30fps, 64*360@30fps

## Run nvBlox without cuvSlam

- Run static TF broadcast
```bash
cd /home/orbbec/Documents/orbbec/opdk/install/isaac_orbbec_launch/share/isaac_orbbec_launch/launch
python base_static_transforms_publisher.py --dev_matrices=/home/orbbec/Documents/orbbec/opdk/install/isaac_orbbec_launch/share/isaac_orbbec_launch/config/dev_matrices_SN1423724335594.yaml
```

- Run 4 cameras launch
```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
ros2 launch orbbec_camera multi_camera_synced.launch.py
```

- Run Nvblox and cuVSLAM
```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
ros2 launch isaac_ros_perceptor_bringup rgbd_perceptor.launch.py config_file:=/home/orbbec/Documents/orbbec/opdk/install/isaac_orbbec_launch/share/isaac_orbbec_launch/param/orbbec_perceptor_detached.yaml
```

- Run Nvblox without cuVSLAM
```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
ros2 launch isaac_ros_perceptor_bringup rgbd_perceptor.launch.py config_file:=/home/orbbec/Documents/orbbec/opdk/install/isaac_orbbec_launch/share/isaac_orbbec_launch/param/orbbec_perceptor_detached.yaml disable_cuvslam:=true
```


```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover
```

- View the output Pose from cuVSLAM, which is of type nav_msgs/msg/Odometry.

```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
ros2 topic echo /visual_slam/tracking/odometry --no-arr
```
