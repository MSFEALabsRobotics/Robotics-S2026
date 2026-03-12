# Basic URDF Tutorial: Two Simple 2-Joint Robots in RViz

This tutorial shows how to:

- create **2 very simple URDF robots**
- launch them in **RViz**
- move their joints using **joint_state_publisher_gui**

It is meant for beginners and uses plain URDF only.

---

# 1) Install required packages

For ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install ros-jazzy-urdf-tutorial
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-rviz2
```

---

# 2) Create a simple package for the files

Go to your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_urdf_tutorial
```

Create folders:

```bash
cd ~/ros2_ws/src/my_urdf_tutorial
mkdir urdf launch rviz
```

---

# 3) Robot 1: Simple 2-joint arm

Create this file:

```bash
nano ~/ros2_ws/src/my_urdf_tutorial/urdf/two_joint_arm.urdf
```

Paste:

```xml
<?xml version="1.0"?>
<robot name="two_joint_arm">

  <!-- Base -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.05"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="gray">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.6 0.05 0.05"/>
      </geometry>
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.1 0.3 0.9 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Link 2 -->
  <link name="link2">
    <visual>
      <geometry>
        <box size="0.5 0.05 0.05"/>
      </geometry>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0.1 0.8 0.2 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Joint 2 -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.6 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

---

# 4) Robot 2: Simple 2-joint robot with different shape

Create:

```bash
nano ~/ros2_ws/src/my_urdf_tutorial/urdf/two_joint_robot2.urdf
```

Paste:

```xml
<?xml version="1.0"?>
<robot name="two_joint_robot2">

  <!-- Base -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.12" length="0.08"/>
      </geometry>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0.2 0.2 0.2 1.0"/>
      </material>
    </visual>
  </link>

  <!-- First arm -->
  <link name="arm1">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.5"/>
      </geometry>
      <origin xyz="0.25 0 0" rpy="0 1.5708 0"/>
      <material name="red">
        <color rgba="0.9 0.2 0.2 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Second arm -->
  <link name="arm2">
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.4"/>
      </geometry>
      <origin xyz="0.2 0 0" rpy="0 1.5708 0"/>
      <material name="yellow">
        <color rgba="0.9 0.9 0.2 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Joint 1 -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm1"/>
    <origin xyz="0 0 0.04" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Joint 2 -->
  <joint name="elbow_joint" type="revolute">
    <parent link="arm1"/>
    <child link="arm2"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.2" upper="1.2" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

---

# 5) Launch file for RViz + joint_state_publisher_gui

Create:

```bash
nano ~/ros2_ws/src/my_urdf_tutorial/launch/display.launch.py
```

Paste:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value='',
        description='Absolute path to robot urdf file'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )

    robot_description = Command(['cat ', LaunchConfiguration('model')])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(['not ', LaunchConfiguration('gui')])
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
```

---

# 6) Small package setup fix

Open:

```bash
nano ~/ros2_ws/src/my_urdf_tutorial/setup.py
```

Use:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_urdf_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='Simple URDF tutorial package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
```

Create resource file:

```bash
mkdir -p ~/ros2_ws/src/my_urdf_tutorial/resource
touch ~/ros2_ws/src/my_urdf_tutorial/resource/my_urdf_tutorial
```

---

# 7) Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select my_urdf_tutorial
source install/setup.bash
```

---

# 8) Run robot 1 in RViz

```bash
ros2 launch my_urdf_tutorial display.launch.py model:=~/ros2_ws/src/my_urdf_tutorial/urdf/two_joint_arm.urdf gui:=true
```

What you should see:

- RViz opens
- `joint_state_publisher_gui` opens
- sliders for `joint1` and `joint2`
- moving the sliders changes the robot pose

---

# 9) Run robot 2 in RViz

```bash
ros2 launch my_urdf_tutorial display.launch.py model:=~/ros2_ws/src/my_urdf_tutorial/urdf/two_joint_robot2.urdf gui:=true
```

Now you should get sliders for:

- `shoulder_joint`
- `elbow_joint`

---

# 10) RViz setup

If the robot does not appear immediately, in RViz:

1. Set **Fixed Frame** to:

```text
base_link
```

2. Add display:
- **RobotModel**

3. Optionally add:
- **TF**

---

# 11) Very simple explanation of URDF parts

A URDF robot is mainly built from:

## Link
A rigid body.

Examples:
- base
- arm
- wheel

## Joint
A connection between two links.

Examples:
- revolute = rotating joint
- fixed = no movement

## Origin
Defines where the child link or visual is placed.

```xml
<origin xyz="0.5 0 0" rpy="0 0 0"/>
```

## Axis
Defines the joint rotation axis.

```xml
<axis xyz="0 0 1"/>
```

## Limit
Defines allowed joint range.

```xml
<limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
```

---

# 12) Why the visual origin is not at zero sometimes

Example:

```xml
<box size="0.6 0.05 0.05"/>
<origin xyz="0.3 0 0" rpy="0 0 0"/>
```

Reason:

- the joint is at the start of the link
- the box length is `0.6`
- so the center of the box must be placed at `0.3`

This makes the link extend forward from the joint.

---

# 13) Common beginner mistakes

## Robot not visible
Usually one of these:

- fixed frame is wrong
- `robot_state_publisher` not running
- URDF path is wrong
- no `RobotModel` display added in RViz

## Sliders move but robot does not move
Usually:

- `joint_state_publisher_gui` is running
- but `robot_state_publisher` is missing

## Weird link positions
Usually:

- wrong `<origin xyz=...>`
- forgetting that geometry is centered on its own origin

---

# 14) Useful checks

Check active nodes:

```bash
ros2 node list
```

You should usually see:

```text
/joint_state_publisher_gui
/robot_state_publisher
/rviz2
```

Check topics:

```bash
ros2 topic list
```

You should see topics like:

```text
/joint_states
/tf
/tf_static
/robot_description
```

Check joint states:

```bash
ros2 topic echo /joint_states
```

---

# 15) Next step after this tutorial

After this basic tutorial, the usual next steps are:

- add a **fixed world link**
- add **more joints**
- replace simple boxes/cylinders with **meshes**
- move from URDF to **xacro**
- later connect to **Gazebo** and **MoveIt**

---

# 16) Quick command summary

Build:

```bash
cd ~/ros2_ws
colcon build --packages-select my_urdf_tutorial
source install/setup.bash
```

Run robot 1:

```bash
ros2 launch my_urdf_tutorial display.launch.py model:=~/ros2_ws/src/my_urdf_tutorial/urdf/two_joint_arm.urdf gui:=true
```

Run robot 2:

```bash
ros2 launch my_urdf_tutorial display.launch.py model:=~/ros2_ws/src/my_urdf_tutorial/urdf/two_joint_robot2.urdf gui:=true
```
