Ubuntu 22.04 - ROS 2 Humble - ArduPilot - Gazebo Setup Guide
This guide provides step-by-step instructions to set up ROS 2 Humble, ArduPilot, and Gazebo Harmonic on Ubuntu 22.04 for drone simulation.

1. Set Locale
Make sure you have a locale that supports UTF-8. If you are in a minimal environment (such as a Docker container), the locale may be something minimal like POSIX. Test with the following settings:



locale  # Check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verify settings
2. Setup ROS 2 Sources
Add the ROS 2 apt repository to your system.

Enable the Ubuntu Universe Repository


sudo apt install software-properties-common
sudo add-apt-repository universe
Add the ROS 2 GPG Key


sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
Add the ROS 2 Repository


echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
3. Install ROS 2 Packages
Update your apt repository caches and install ROS 2 packages.

Update and Upgrade


sudo apt update
sudo apt upgrade
Install ROS 2 Desktop (Recommended)


sudo apt install ros-humble-desktop
Install ROS 2 Base (Bare Bones)


sudo apt install ros-humble-ros-base
Install Development Tools

sudo apt install ros-dev-tools
4. Environment Setup
Source the ROS 2 setup script to configure your environment.



# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
5. Test ROS 2 Installation
Run the talker-listener example to verify the installation.

Run C++ Talker


source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
Run Python Listener


source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
You should see the talker publishing messages and the listener receiving them. This verifies both the C++ and Python APIs are working properly.

6. Set Up ArduPilot with ROS 2
Clone the required repositories and set up the workspace.

Create Workspace and Clone Repositories


mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
Update Dependencies


cd ~/ardu_ws
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
Install MicroXRCEDDSGen Build Dependency


cd
sudo apt install default-jre
cd ~/ardu_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
Test MicroXRCEDDSGen Installation


source ~/.bashrc
microxrceddsgen -help
7. Build the Workspace
Build the ROS 2 workspace.



cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests
colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+
python3 -m pip install --user pexpect
python3 -m pip install --user future
colcon build --packages-up-to ardupilot_dds_tests  # Repeat until no failures
source ./install/setup.bash
8. Launch ArduPilot SITL
Launch the ArduPilot SITL simulation.



cd ~/ardu_ws
colcon build --packages-up-to ardupilot_sitl
source install/setup.bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
9. Interact with ArduPilot via ROS 2 CLI
Use the ROS 2 CLI to interact with ArduPilot.



source ~/ardu_ws/install/setup.bash
ros2 node list
ros2 node info /ap
ros2 topic echo /ap/geopose/filtered
If ROS 2 topics aren’t being published, ensure the ArduPilot parameter DDS_ENABLE is set to 1 and reboot the launch.



export PATH=$PATH:~/ardu_ws/src/ardupilot/Tools/autotest
sim_vehicle.py -w -v ArduPlane --console -DG --enable-dds
param set DDS_ENABLE 1
Ensure the ArduPilot parameter DDS_DOMAIN_ID matches your environment variable ROS_DOMAIN_ID. The default is 0 for ArduPilot.

10. Install Gazebo
Install Gazebo Harmonic (recommended) or Gazebo Garden.

Clone Required Repositories


cd ~/ardu_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
Set Gazebo Version
Set the Gazebo version in your ~/.bashrc file.



export GZ_VERSION=harmonic
Update ROS Dependencies


cd ~/ardu_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r
11. Build and Run Tests
Build the workspace and run tests.



cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup
If the build fails, install missing dependencies:



sudo apt install libgz-cmake3-dev libgz-common5-dev libgz-sim7-dev libgz-math7-dev libgz-msgs9-dev libgz-transport12-dev libgz-tools2-dev libgz-utils2-dev
sudo apt update
sudo apt install gz-cmake
sudo apt update && sudo apt upgrade -y
sudo apt install curl -y
curl -sSL https://get.gazebosim.org | sh
sudo apt install gz-cmake3
pip install gz-cmake3
sudo apt install python3-vcstool python3-colcon-common-extensions git wget
colcon build --cmake-args -DBUILD_TESTING=ON
cd ~/ardu_ws/
colcon build --cmake-args -DBUILD_TESTING=ON
sudo apt update
sudo apt install libgz-cmake3-dev libgz-common5-dev libgz-sim8-dev libgz-math7-dev libgz-msgs10-dev libgz-transport13-dev libgz-tools2-dev libgz-utils2-dev
gz sim --version
colcon build --packages-up-to ardupilot_gz_bringup
12. Launch Gazebo Simulation
Launch the Gazebo simulation with ArduPilot.



cd ~/ardu_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
This launch file starts ArduPilot SITL, Gazebo, and RViz with a single command.

13. MAVProxy
To test and fly around, launch a MAVProxy instance in another terminal.



mavproxy.py --console --map --aircraft test --master=:14550
This guide should help you set up ROS 2 Humble, ArduPilot, and Gazebo Harmonic on Ubuntu 22.04. Let me know if you encounter any issues!
