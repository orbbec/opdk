
# OPDK System Restoration Instruction

## 1. Software Environment Preparation

> Note: The purpose of this document is to provide a re-flashing guide for Orbbec Perceptor Development Kit（OPDK） users in case the system firmware crashes and the AGX Orin is rendered unbootable during the development process. To ensure optimal performance and compatibility, the Orbbec Perceptor Development Kit requires a properly configured development environment on the NVIDIA AGX Orin platform. Follow the steps below to set up the environment, including the installation of the necessary operating system, libraries, and firmware requirements.

### 1.1 Reflash AGX

* Please follow Nvidia support documents to recover Ubuntu system (22.04+) and Jetpack 6.1
* [https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/howto.html](1) Please follow Nvidia support documents to recover Ubuntu system (22.04+) and Jetpack 6.1 2) https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/howto.html)

### 1.2 Library Dependencies

Install the following libraries to enable interaction with the Orbbec Gemini 335L cameras and support robotics functionalities:

* OrbbecSDK & OrbbecViewer: Download and install the latest Orbbec SDK (2.1.1+), following the installation instructions provided by Orbbec.

  * Orbbec SDK GitHub Repository: [Orbbec SDK GitHub](https://github.com/orbbec/OrbbecSDK_v2)
  * Orbbec SDK Installation Guide: [Orbbec SDK Installation Guide](https://orbbec.github.io/OrbbecSDK/doc/tutorial/Chinese/Installation_guidance.html)
* Install the ROS 2 wrapper (ver. 2.1.1+) provided by Orbbec to enable seamless integration with ROS 2 applications.

  * Orbbec ROS 2 Wrapper GitHub Repository: [Orbbec ROS 2 Wrapper GitHub](https://github.com/orbbec/OrbbecSDK_ROS2)
  * Orbbec ROS 2 Wrapper User Manual: [Orbbec ROS 2 Wrapper User Manual](https://www.orbbec.com.cn/index/Gemini330/info.html?cate=119&id=157)
* Install NVIDIA Isaac Perceptor (3.2) to leverage advanced perception capabilities

  * Isaac Perceptor GitHub Repository: [Isaac Perceptor GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_perceptor)
  * IsaacPerceptor Installation Guide: [Isaac Perceptor Installation Guide](https://developer.nvidia.com/isaac/perceptor)

### 1.3 System Updates and Firmware Requirements

Ensure that the Orbbec Gemini 335L camera firmware is updated to version 1.4.00 or higher to guarantee compatibility and access to all camera features.

* Firmware Download: [Orbbec Firmware Download](https://github.com/orbbec/OrbbecFirmware/releases)
* Firmware Upgrade Guide: [Orbbec Firmware Upgrade Instructions](https://www.orbbec.com/docs/g330-update-firmware/)

## 2. Installation Commandline

### 2.1 Install Isaac license

```bash

wget-qO-https://isaac.download.nvidia.com/isaac-ros/repos.key | sudoapt-keyadd-

grep-qxF"deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release-cs) release-3.0"/etc/apt/sources.list || echo"deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release-cs) release-3.0" | sudotee-a/etc/apt/sources.list

sudoaptupdate

```

### 2.2 Install Ros2

```bash

sudoaptupdate && sudoaptinstallcurlgnupglsb-release-y

sudocurl-sSLhttps://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudoapt-keyadd-

echo"deb http://packages.ros.org/ros2/ubuntu $(lsb_release-cs) main" | sudotee/etc/apt/sources.list.d/ros2-latest.list

sudoaptupdate

```

### 2.3 Install Ros2 Humble and colcon

```bash

sudoaptinstallros-humble-desktop-y

sudoaptinstallpython3-colcon-common-extensions-y

```

### 2.4 Install dependencies

```bash

sudoaptinstallvpi3-devlibnvvpi3ros-humble-isaac-ros-nitros\

ros-humble-isaac-ros-managed-nitrosros-humble-isaac-ros-nitros-image-type\

ros-humble-isaac-ros-nitros-camera-info-type\

libbenchmark-devros-humble-foxglove-bridgeros-humble-nav2-costmap-2dlibgoogle-glog-dev\

libgflags-devnlohmann-json3-dev\

ros-humble-image-transportros-humble-image-publisherros-humble-camera-info-manager\

ros-humble-diagnostic-updaterros-humble-diagnostic-msgsros-humble-statistics-msgs\

ros-humble-backward-roslibdw-devros-humble-image-transport\

ros-humble-image-transport-pluginsros-humble-compressed-image-transport\

ros-humble-rqt-tf-tree\

ros-humble-rqt-image-viewros-humble-rviz2clang-format\

git-lfs-y

```

### 2.5 Git Setup

```bash

gitlfsinstall

gitconfig--globaluser.name"user"

gitconfig--globaluser.email“user@email”

mkdir~/.ssh

cd~/.ssh

ssh-keygen-trsa-C“user@email”

```

Add id_rsa.pub key into Github account

### 2.6 Get Source Code

```bash

mkdir/home/orbbec/Documents/orbbec-p

cd/home/orbbec/Documents/orbbec

gitclonegit@github.com:orbbec/opdk.git

cdopdk

gitsubmoduleupdate--init–recursive

```

### 2.7 Install camera udev support

```bash

cd/home/orbbec/Documents/orbbec/opdk/src/OrbbecSDK_ROS2/orbbec_camera/scripts

sudobashinstall_udev_rules.sh

sudoudevadmcontrol--reload-rules && sudoudevadmtrigger

```

### 2.8 Setup environment variables

```bash

vi~/.bashrc

source/opt/ros/humble/setup.bash

exportROS_DOMAIN_ID=55

```

### 2.9 Install Jetson-stats

```bash

sudoaptinstallpip-y

sudopip3install--no-cache-dir-v-Ujetson-stats

jetson_release-v

```

### 2.10 Install CUDA and setup environment variables

```bash

sudoaptinstallcuda-y

echo'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc

echo'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc

echo'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc

```

### 2.11 Compile

```bash

cd/home/orbbec/Documents/orbbec/opdk

colconbuild--packages-skipnvblox_test_datanvblox_test--event-handlersconsole_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

```

### 2.12 System Optimization

FastDDS: [https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/docs/fastdds_tuning.md](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/docs/fastdds_tuning.md)

usbfs_memory:

```bash

sudovi/etc/systemd/system/usbfs_memory_fix.service


[Unit]

Description=SetUSBFSmemorylimit

After=sysinit.target


[Service]

Type=oneshot

ExecStart=/bin/bash-c"echo 128 | tee /sys/module/usbcore/parameters/usbfs_memory_mb"

RemainAfterExit=yes


[Install]

WantedBy=multi-user.target

```

### 2.13 Reboot

## 3. Reload Calibration File

OPDK provides calibration data download service on Github. The customer can find the calibration data of your system by its SN. Checking the SN can use the following command.

```bash

cat/sys/firmware/devicetree/base/serial-number

```

The calibration file can be find [here](https://github.com/orbbec/OrbbecSDK_ROS2/tree/9caaa78c0adabebf73418e10912599002daf1792/isaac_orbbec_launch/config). The customer can load the correct file when start OPDK nodes, following the instruction [here](https://github.com/orbbec/opdk).

## 4. Readiness Check

### 4.1 OrbbecViewer

Customer can always use OrbbecViewer to check the status of all Orbbec cameras.

Github: [https://github.com/orbbec/OrbbecSDK_v2](https://github.com/orbbec/OrbbecSDK_v2)

How to use OrbbecViewer：[https://github.com/orbbec/OrbbecSDK_v2/blob/main/docs/tutorial/orbbecviewer.md](https://github.com/orbbec/OrbbecSDK_v2/blob/main/docs/tutorial/orbbecviewer.md)

### 4.2 Check Camera USB Connection in Ros2

```bash

ros2runorbbec_cameralist_devices_node

```

### 4.3 Rviz2

When the system is started and all topics are published (refer to [OPDK Github Readme](https://github.com/orbbec/opdk)), customer can use  rviz2 to check the topics.

```bash

rviz2-d/home/orbbec/Documents/orbbec/opdk/install/isaac_orbbec_launch/share/isaac_orbbec_launch/param/perceptor_odom.rviz

```
