# Ubuntu 22.04 - ROS 2 Humble - ArduPilot - Gazebo Setup Guide

This guide provides step-by-step instructions to set up ROS 2 Humble, ArduPilot, and Gazebo Harmonic on Ubuntu 22.04 for drone simulation.

## 1. Set Locale

Make sure you have a locale that supports UTF-8. If you are in a minimal environment (such as a Docker container), the locale may be something minimal like POSIX. Test with the following settings:

```sh
locale  # Check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verify settings
```

## 2. Setup ROS 2 Sources

Add the ROS 2 apt repository to your system.

### Enable the Ubuntu Universe Repository

```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Add the ROS 2 GPG Key

```sh
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Add the ROS 2 Repository

```sh
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 3. Install ROS 2 Packages

Update your apt repository caches and install ROS 2 packages.

### Update and Upgrade

```sh
sudo apt update
sudo apt upgrade
```

### Install ROS 2 Desktop (Recommended)

```sh
sudo apt install ros-humble-desktop
```

### Install ROS 2 Base (Bare Bones)

```sh
sudo apt install ros-humble-ros-base
```

### Install Development Tools

```sh
sudo apt install ros-dev-tools
```

## 4. Environment Setup

Source the ROS 2 setup script to configure your environment.

```sh
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash

# Add to bash file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 5. Test ROS 2 Installation

Run the talker-listener example to verify the installation.

### Run C++ Talker

```sh
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

### Run Python Listener

```sh
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see the talker publishing messages and the listener receiving them. This verifies both the C++ and Python APIs are working properly.

## 6. Set Up ArduPilot with ROS 2

Clone the required repositories and set up the workspace.

### Create Workspace and Clone Repositories

```sh
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
```

### Update Dependencies

```sh
cd ~/ardu_ws
sudo apt update
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### Install MicroXRCEDDSGen Build Dependency

```sh
cd
sudo apt install default-jre
cd ~/ardu_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
```

### Test MicroXRCEDDSGen Installation

```sh
source ~/.bashrc
cd
microxrceddsgen -help
```

## 7. Build the Workspace

Build the ROS 2 workspace.

```sh
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests
colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+
sudo apt install python3-pip
python3 -m pip install --user pexpect
python3 -m pip install --user future
colcon build --packages-up-to ardupilot_dds_tests  # Repeat until no failures
source ./install/setup.bash
cd
```

## 8. Launch ArduPilot SITL

Launch the ArduPilot SITL simulation.

```sh
source /opt/ros/humble/setup.bash # You don't have to source everytime if you have added to bash file 
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_sitl
source install/setup.bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad
```

## 9. Interact with ArduPilot via ROS 2 CLI

Use the ROS 2 CLI to interact with ArduPilot.

```sh
source ~/ardu_ws/install/setup.bash
ros2 node list
ros2 node info /ap
ros2 topic echo /ap/geopose/filtered
```

If ROS 2 topics aren’t being published, ensure the ArduPilot parameter `DDS_ENABLE` is set to 1 and reboot the launch.

```sh
export PATH=$PATH:~/ardu_ws/src/ardupilot/Tools/autotest
sim_vehicle.py -w -v ArduPlane --console -DG --enable-dds
param set DDS_ENABLE 1
```

Ensure the ArduPilot parameter `DDS_DOMAIN_ID` matches your environment variable `ROS_DOMAIN_ID`. The default is 0 for ArduPilot.

## 10. Install Gazebo

Install Gazebo Harmonic (recommended) or Gazebo Garden.

### Clone Required Repositories

```sh
cd ~/ardu_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
```

### Set Gazebo Version

Set the Gazebo version in your `~/.bashrc` file.

```sh
export GZ_VERSION=harmonic
```

### Update ROS Dependencies

```sh
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r
```

## 11. Build and Run Tests

Build the workspace and run tests.

```sh
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup
```

If the build fails, install missing dependencies:

```sh
sudo apt update && sudo apt upgrade -y
sudo apt install curl -y
curl -sSL https://get.gazebosim.org | sh
sudo apt install gz-cmake3
colcon build --packages-up-to ardupilot_gz_bringup
```

## 12. Launch Gazebo Simulation

Launch the Gazebo simulation with ArduPilot.

```sh
cd ~/ardu_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

This launch file starts ArduPilot SITL, Gazebo, and RViz with a single command.

## 13. MAVProxy

To test and fly around, launch a MAVProxy instance in another terminal.

```sh
mavproxy.py --console --map --aircraft test --master=:14550
```

This guide should help you set up ROS 2 Humble, ArduPilot, and Gazebo Harmonic on Ubuntu 22.04. Let me know if you encounter any issues!
````
