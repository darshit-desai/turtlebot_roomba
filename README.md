# turtlebot_roomba
A repo for walking the turtlebot around an obstacle map

```bash
# Source to ROS2 HUMBLE
source /opt/ros/humble/setup.bash
# Make your ros2 workspace
mkdir ros2_ws
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws
git clone https://github.com/darshit-desai/

# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select tbot_roomba
# After successfull build source the package
. install/setup.bash

```
