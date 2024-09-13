# Manipulator_RL_reach_Policy-Ditillation
Lightening the AI model to reach the target point of the robot arm with policy Distillation

#### Notice
>this repository is based on CUN-bjy/policy-distillation-baselines

#### Evironment
>os : Ubuntu 18.04 LTS<br/>
>python V : 3.6<br/>
>robot : Turtlebot3<br/>
>Simulator : GAZEBO<br/>
>ROS version : Melodic

# TurtleBot Jetson Nano + OpenCR Setup

## 1. Jetson Setup

```bash
# Update and upgrade the system
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y chrony ntpdate build-essential curl nano

# Sync time with NTP
sudo ntpdate ntp.ubuntu.com

# Add ROS repository to sources list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add ROS package keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update and install ROS Melodic
sudo apt-get update -y
sudo apt-get install -y ros-melodic-ros-base

# Install ROS dependencies
sudo apt install python-rosdep
sudo rosdep init
rosdep update

# Create ROS workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Install required ROS packages for TurtleBot3
sudo apt install ros-melodic-rosserial-python ros-melodic-tf ros-melodic-hls-lfcd-lds-driver ros-melodic-turtlebot3-msgs ros-melodic-dynamixel-sdk

# Clone TurtleBot3 repository
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git

# Remove unnecessary folders from TurtleBot3
cd ~/catkin_ws/src/turtlebot3
rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/

# Source ROS setup script
source /opt/ros/melodic/setup.sh

# Build workspace
cd ~/catkin_ws && catkin_make

# Source bashrc
source ~/.bashrc
```

## 2. OpenCR Setup
```bash
# Add ARM architecture and install ARM libraries
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf

# Set OpenCR port and model
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=waffle

# Download OpenCR firmware and extract it
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd ./opencr_update

# Give write permissions to the port
sudo chmod a+rw /dev/ttyACM0

# Update OpenCR firmware
sudo ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr

# Download and apply udev rules for OpenCR
wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
