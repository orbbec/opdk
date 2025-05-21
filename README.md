# OPDK

## Table of Contents

- [OPDK](#opdk)
  - [Table of Contents](#table-of-contents)
  - [Installation Instructions](#installation-instructions)
  - [Build Workspace](#build-workspace)
  - [Getting start](#getting-start)
    - [Run single camera with cuVSLAM and Nvblox](#run-single-camera-with-cuvslam-and-nvblox)
  - [FAQ](#faq)

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

- Run all opdk nodes

```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
ros2 launch isaac_orbbec_launch orbbec_perceptor.launch.py dev_matrices:=config/dev_matrices_SN1423724335594.yaml
```

**Another step-by-step execution method**

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

- Run cuVSLAM without Nvblox

```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
ros2 launch isaac_ros_perceptor_bringup rgbd_perceptor.launch.py config_file:=/home/orbbec/Documents/orbbec/opdk/install/isaac_orbbec_launch/share/isaac_orbbec_launch/param/orbbec_perceptor_detached.yaml disable_nvblox:=true
```

![This is a local image](./image/opdk_rviz.png "Optional title")

- Visualize the TF frame hierarchy using rqt_tf_tree with forced discovery.

```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover
```

- View the output Pose from cuVSLAM, which is of type nav_msgs/msg/Odometry.

```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
ros2 topic echo /visual_slam/tracking/odometry --no-arr
```

### Run single camera with cuVSLAM and Nvblox

* Run single usb camera launch

```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py config_file_path:=/home/orbbec/Documents/orbbec/opdk/src/OrbbecSDK_ROS2/orbbec_camera/config/single_camera_params.yaml
```

* Or run single net camera launch

```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
# Write the IP address of the net camera into net_device_ip
ros2 launch orbbec_camera gemini_330_series.launch.py config_file_path:=/home/orbbec/Documents/orbbec/opdk/src/OrbbecSDK_ROS2/orbbec_camera/config/single_camera_params.yaml enumerate_net_device:=true net_device_ip:=192.168.1.10 net_device_port:=8090
```

* Run Nvblox and cuVSLAM

```bash
cd /home/orbbec/Documents/orbbec/opdk
source install/setup.bash
ros2 launch isaac_ros_perceptor_bringup rgbd_perceptor.launch.py config_file:=/home/orbbec/Documents/orbbec/opdk/install/isaac_orbbec_launch/share/isaac_orbbec_launch/param/orbbec_single_camera_perceptor_detached.yaml
```

* Run rviz2

```bash
rviz2 -d /home/orbbec/Documents/orbbec/opdk/install/isaac_orbbec_launch/share/isaac_orbbec_launch/param/single_camera_odom.rviz
```

## FAQ

1. How to check the SN number of Orin device?

```bash
cat /sys/firmware/devicetree/base/serial-number
```

2. How to check the USB port number and SN number of the camera?

```bash
ros2 run orbbec_camera list_devices_node
```

3. How to switch the camera extrinsics file when starting launch?

```bash
ros2 launch isaac_orbbec_launch orbbec_perceptor.launch.py dev_matrices:=config/dev_matrices_SN1423724335594.yaml
```

Replace `dev_matrices_SN1423724335594.yaml` with the external reference yaml file of the current device.

For example:

```bash
ros2 launch isaac_orbbec_launch orbbec_perceptor.launch.py dev_matrices:=config/dev_matrices_SN1423624327954.yaml
```

4. How does the camera use specific camera parameter configuration yaml files?

For example, if you want to run the configuration of cuvslam+nvblox 640*360 60fps, open `multi_camera_synced.launch.py` and replace `camera_params.yaml` with `camera_params_cuvslam_nvblox-640_360_60fps.yaml`

![This is a local image](./image/multi_camera_synced.png "Optional title")

5. How to determine the topic frame rate?

For example, if you want to check the frame rate of the depth stream of left_camera:

```bash
ros2 topic hz /left_camera/depth/image_raw
```

6. Nvblox topic content judgment

[Isaac ROS Nvblox Topics and Services](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/topics_and_services.html)

a. `/nvblox_node/color_layer`：Pointcloud visualizing color voxels.

![This is a local image](./image/color_layer.png "Optional title")

b.`/nvblox_node/dynamic_occupancy_layer`：A pointcloud of the people/dynamic occupancy map (only voxels with occupation probability > 0.5).

c. `/nvblox_node/combined_esdf_pointcloud`：A pointcloud of the combined static and people/dynamic 2D ESDF (minimal distance of both), with intensity as the metric distance to the nearest obstacle or person.

![This is a local image](./image/combined_esdf_pointcloud.png "Optional title")

Effect of subscribing to two topics at the same time：

![This is a local image](./image/nvblox_image.png "Optional title")

7. VSLAM content judgment

[Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#quickstart)

The normal operation of VSLAM mainly depends on whether the odom data is updated normally:

```bash
ros2 topic echo /visual_slam/tracking/odometry
```

8. OrbbecSDK log storage and analysis

Modify `OrbbecSDKConfig_v2.0.xml` in the OrbbecSDK_ROS2 package and change FileLogLevel to 0

![This is a local image](./image/OrbbecSDKConfig_v2.0.png "Optional title")

Then recompile and start the camera, and you can see the camera log file `OrbbecSDK.log.txt` in the Log folder in the opdk folder

9. If you find that the ros2 topic hz frame rate is not as expected, check whether the following optimization points are still effective

Optimizing FastDDS：[https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/docs/fastdds_tuning.md](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/docs/fastdds_tuning.md)

Increase the usb cache to 128MB:

```bash
sudo vi /etc/systemd/system/usbfs_memory_fix.service
```

add the following to usbfs_memory_fix.service

```plaintext
[Unit]
Description=Set USBFS memory limit
After=sysinit.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c "echo 128 | tee /sys/module/usbcore/parameters/usbfs_memory_mb"
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Reload systemd

```bash
sudo systemctl daemon-reload
```

Start the service to verify operation

```bash
sudo systemctl start usbfs_memory_fix.service
```

Check whether it works

```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

Enable the service to start automatically at boot

```bash
sudo systemctl enable usbfs_memory_fix.service
```

Verify service status

```bash
sudo systemctl status usbfs_memory_fix.service
```

After the system restarts, check again

```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

10. Put nvblox, cuvslam, and orbbec_camera into the same container

Open the orbbec_perceptor_detached.yaml file and modify the container_name and attach_to_container

```bash
nvblox_config:
  container_name: 'nvblox_container'
  attach_to_container: false
cuvslam_config:
  container_name: 'cuvslam_container'
  attach_to_container: false
```

to:

```bash
nvblox_config:
  container_name: 'shared_orbbec_container'
  attach_to_container: true
cuvslam_config:
  container_name: 'shared_orbbec_container'
  attach_to_container: true
```

11. How can I reflash the OPDK if the system firmware crashes and the AGX Orin becomes unbootable during development?

Please refer to the [OPDK System Restoration Instruction](docs/OPDK%20System%20Restoration%20Instruction/README.md) detailed steps.

12. Where can I find technical support after reflashing OPDK?

If the issue is related to hardware or firmware, and OPDK is unable to stream data or provide ROS 2 topics properly (see [Section 4](docs/OPDK%20System%20Restoration%20Instruction/README.md#readiness-check)): Readiness Check in the OPDK System Restoration Instruction), users should contact Orbbec Tech Support for assistance.

If the issue pertains to NVIDIA Isaac Perceptor software while OPDK is able to stream data and provide ROS 2 topics correctly (see [Section 4](docs/OPDK%20System%20Restoration%20Instruction/README.md#readiness-check)): Readiness Check in the OPDK System Restoration Instruction), users should contact NVIDIA Tech Support for assistance.