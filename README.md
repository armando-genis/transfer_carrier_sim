# transfer_carrier_sim


## Install
```bash
sudo apt-get install libeigen3-dev
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
sudo apt install ros-<ros2-distro>-xacro
sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs
sudo apt install ros-<ros2-distro>-ackermann-msgs
sudo apt install ros-<distro>-ros2-control ros-<distro>-ros2-controllers
sudo apt install ros-<distro>-controller-manager
sudo apt install ros-<distro>-laser-geometry

```

 ## Set up ROS2
```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
source install/setup.bash
```

# Model description
```bash
colcon build --packages-select niagara_model
source install/setup.bash
ros2 launch transfer_carrier_description gazebo.launch.py #For launching with gazebo and rviz
ros2 launch transfer_carrier_description display.launch.py
```
 
# Obstacle detector based lidar node
```bash
colcon build --packages-select lidar_detector
source install/setup.bash
ros2 launch lidar_detector Obstacles.launch.py
```

# Mux velocity node
```bash
colcon build --packages-select mux_velocity
source install/setup.bash
ros2 launch mux_velocity mux.launch.py
```
 
