# turtlebot_roomba
A repo for walking the turtlebot around an obstacle map using randomized exploration.

The repo makes a node for turtlebot to roam around the obstacle map and take turns upon encountering an
obstacle like a roomba vacuum cleaner. The turns are completely randomized using the random stl library.

# Build and Run Instructions
```bash
# Source to ROS2 HUMBLE
source /opt/ros/humble/setup.bash
# Make your ros2 workspace
mkdir ros2_ws
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws
git clone https://github.com/darshit-desai/turtlebot_roomba.git
# Install process
## Ensure you have necessary packages installed
rosdep install -i --from-path src --rosdistro humble -y
sudo apt -y install ros-humble-desktop-full
sudo apt -y install ros-humble-gazebo-ros-pkgs
sudo apt -y install ros-humble-turtlebot3*
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select tbot_roomba
# After successfull build source the package
. install/setup.bash
# To launch the world run the ros node without bagging the rosbag
export TURTLEBOT3_MODEL=burger && ros2 launch tbot_roomba launch.py
# To record the rosbag while launching and running the ros node
export TURTLEBOT3_MODEL=burger && ros2 launch tbot_roomba launch.py rosbag_record:=true
# To check the Rosbag details
ros2 bag info walker_bag/
# To play the Rosbag
ros2 bag play walker_bag/walker_bag_0.db3
```

### CppCheck & CppLint
```bash
# Use the below command for cpp check by moving to root directory of your workspace
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) --check-config  &> Results/cppcheck.txt

# Use the below command for cpp lint by moving to root directory of your workspace 
cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> Results/cpplint.txt 

## The results of both are present in results folder insider beginner_tutorials directory
```

# Results
The following gif shows an example run of the turtlebot3 in the map,

![](Results/Kazam_screencast_00095.gif)



