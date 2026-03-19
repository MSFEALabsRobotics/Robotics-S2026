# CR10 MoveIt Setup and Joint Velocity Fix

## Install required packages

```bash
sudo apt update
sudo apt install ros-jazzy-controller-manager ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt-get install ros-jazzy-moveit
```

## Start MoveIt

```bash
ros2 launch dobot_moveit moveit_demo.launch.py
```

## If planning gives a velocity error

Open the joint limits file:

```bash
gedit ~/ros2_ws/src/DOBOT_6Axis_ROS2_V4/cr10_moveit/config/joint_limits.yaml
```

Replace the contents with:

```yaml
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed

default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint2:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint3:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint4:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint5:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint6:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
```

## Rebuild the package

```bash
cd ~/ros2_ws
colcon build --packages-select cr10_moveit
```

## Source the workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Launch MoveIt again

```bash
ros2 launch dobot_moveit moveit_demo.launch.py
```


