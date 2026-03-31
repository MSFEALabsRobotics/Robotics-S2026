# ROBOTICS PROJECT Part1:

## Inverse Kinematics and End-Effector Trajectory Control of the Dobot CR10 in RViz

### Project Overview
The objective of this project is to develop and implement an inverse kinematics (IK) model for the Dobot CR10 robotic manipulator and visualize its motion in RViz. The project focuses on building the kinematic solution from first principles, without relying on high-level motion-planning frameworks such as MoveIt or any other similar full-stack solver.

The main goal is to understand and implement the relationship between the robot’s end-effector pose and its joint variables, then use this model to generate and visualize motion trajectories directly in RViz.

### Main Constraints
To ensure that the project emphasizes robot kinematics and low-level control concepts, the following restrictions apply:
- MoveIt and similar full-stack motion-planning packages are not allowed.
- Only simple libraries may be used, such as libraries for:
  - mathematical computations,
  - matrix operations,
  - symbolic modeling,
  - basic robot modeling.
- The Joint State Publisher GUI will be turned off.
- Instead, joint values must be generated and published through a custom ROS 2 node, which will command the robot model in RViz, on the Joints States topic.

### Project Requirements

#### 1. Inverse Kinematics Model
An inverse kinematics model for the Dobot CR10 must be developed.

This model should compute the required joint angles for a desired end-effector position in Cartesian space.

The implementation should include:
- modeling the robot geometry,
- defining the robot kinematic chain,
- deriving or implementing the forward kinematics,
- developing an inverse kinematics solution,
- validating that the computed joint angles correctly place the end effector at the desired target.

#### 2. RViz Visualization Through a Custom Node
Instead of using GUI sliders, a ROS 2 node must be developed to publish joint values to RViz.

This node will:
- generate joint states,
- publish them to the appropriate topic,
- continuously update the robot posture,
- allow the Dobot CR10 model to move based on the IK results.

This demonstrates how robot motion can be controlled programmatically rather than manually.

#### 3. End-Effector Trace Line in RViz
A trace line must be implemented in RViz to show the path followed by the end effector during motion.

This trace should visually represent the trajectory of the end effector over time.

It may be implemented using a suitable RViz visualization message such as:
- a line strip marker, or
- a path-style message or any other custom workflow you like.

The trace should update continuously as the robot moves.

---

### Required Motion Functions

#### Function 1: Straight-Line Motion Between Two Points
A function must be developed to move the end effector in a straight line between two specified points:
- starting point A
- ending point B

Both A and B are random but must be reachable points within the robot workspace.

The function should:
- accept points A and B as inputs,
- interpolate intermediate Cartesian points along the straight-line segment,
- apply inverse kinematics at each intermediate point,
- publish the resulting joint values,
- produce visible straight-line motion in RViz,
- display the corresponding end-effector trace.

This function demonstrates Cartesian trajectory generation and IK-based tracking.

#### Function 2: Circular Motion Parallel to the Ground
A second function must be developed to move the end effector along a circular trajectory that is parallel to the ground.

The circle should be:
- centered at point A
- with radius R

Both A and R are random values, provided that the resulting circle remains reachable by the robot.

The function should:
- accept center point A and radius R as inputs,
- generate circular Cartesian waypoints,
- keep the trajectory parallel to the ground,
- solve inverse kinematics for each point,
- publish the corresponding joint states,
- display the motion and its trace in RViz.

A typical circular path can be defined parametrically while maintaining a constant height.

---

### Expected Deliverables
The final project should include:
1. A kinematic model of the Dobot CR10
2. An inverse kinematics implementation
3. A ROS 2 node for publishing joint states
4. RViz visualization of robot motion
5. A trace-line visualization of the end-effector path
6. Implementation of the two motion functions:
   - straight-line motion from A to B,
   - circular motion around A with radius R

**EACH TEAM SHOULD DO A 10-15min VIDEO RECORDING (TEAMS OR WEBEX OR SIMILAR) AND SHARE A LINK PRESENSITNG THE SOLUTION, EXPLAINING IT IN REAL TIME WHILE DOING THE DEMO IN RVIZ**

# ROBOTICS PROJECT Part2:

## Pick-and-Place of a Parametric Cube Object Using the Dobot CR10

### Project Overview
The objective of this project is to develop a ROS-based robotic pick-and-place application for the Dobot CR10 robot. You will develop it in a Simulated/online manner, but it will be implemented on the physical robot the day of the presentation when the parameters are given.

A cube of side length c is placed on a first table, where the coordinates (x, y, z) define the position of the center of the cube, and α defines its orientation on the table. You should plan the required grasp and motion, taking into effect the end effector, pick the cube, and place it at a mirror position on a second table located symmetrically relative to the robot.

And h is the height of robot from the ground.

The project must be prepared in simulation and through a ROS node structure that will be executed using physical values on the real robot. At the beginning of the node, the parameters c, x, y, z, α, h must be easy to modify.

### Project Requirements

#### 1. Parametric Object Definition
The cube must be defined using the following parameters:
- c: cube side length
- x, y, z: coordinates of the cube center
- α: cube orientation on the table
- h: Height of the base of the robot

These parameters must be adjustable directly at the beginning of the ROS node so the same software can be tested with different physical values later.

#### 2. Simulation Environment (not need for submission, but helps planning)
A simulation scene must be prepared containing:
- the Dobot CR10 robot,
- the first table holding the cube,
- the second table where the cube will be placed,
- the cube object itself.

You can use, move it, Rviz, any other custom simulator, but not the Physical Robot

#### 3. Pick Operation
The available gripper in the lab is the ROBOTIQ 2F-85

To be taken into consideration when modeling it on the robot (information available online, URDF or other…)

<img width="483" height="439" alt="image" src="https://github.com/user-attachments/assets/2ac3f282-d98d-484c-ab76-5751a8c424ac" />

#### 4. Mirror Placement
After grasping the cube, the robot must move it to the second table and place it at the mirrored position. The target pose must be computed from the source cube pose according to the chosen mirror rule.

#### 5. ROS Node Implementation
A ROS node must be developed to:
- define the input parameters c, x, y, z, α, h
- when running this node the robot will physically pick and place the object
