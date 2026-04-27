# Robotics Project: Vision-Based Cube Pick-and-Place Using QR Code Detection

## Project Objective

In this project, you will develop a physical robotic pick-and-place system using a real robot, a camera, and QR code detection. The robot must detect a cube placed on the lab table, estimate its position and orientation using a QR code attached to the cube, pick it up, rotate it, and place it back on the table.

The project focuses on practical robot integration, computer vision, coordinate transformation, safety, and physical testing.

---

## Project Description

You are required to prepare a cube object and attach or print a QR code clearly on its top surface. The QR code will be used by a camera system to detect the cube, estimate its position, and determine its rotation angle relative to the camera frame or table reference frame.

You may use any suitable camera, such as a webcam, USB camera, laptop camera, or external camera module. The camera must be physically integrated into the system during testing. You may mount it above the workspace, attach it near the robot, or place it in another suitable fixed position. At the end of the project, the camera setup must be safely removed and the lab setup restored.

The table height is constant and will be based on the lab table. Therefore, the vertical position of the cube can be assumed known. Your vision system should mainly estimate the cube’s XY translation and rotation angle.

---

## Required System Behavior

The system should perform the following main steps:

1. Detect the QR code on the cube using the camera.
2. Extract the cube position in the camera image.
3. Estimate the cube’s XY translation relative to the robot or workspace reference frame.
4. Estimate the cube’s rotation angle.
5. Move the robot slowly toward the cube.
6. Pick up the cube safely.
7. Lift the cube from the table.
8. Rotate the cube.
9. Place the cube back on the table.

The best-case scenario is that the robot picks the cube, lifts it, rotates it, and places it again on the table in a controlled and repeatable way.

---

## QR Code Detection Requirement

You must implement a QR code detection system. This part can first be developed and tested offline without using the robot.

The QR code detection program should be able to:

* Detect the QR code from a camera image or video stream.
* Identify the QR code corner points.
* Estimate the center of the QR code.
* Compute the cube’s rotation angle.
* Estimate the required XY translation needed for the robot to align with the cube.

The output of the QR detection system should be usable by the robot control system. At minimum, it should provide:

```text
x_position
y_position
rotation_angle
detection_status
```

You should clearly explain how the image coordinates are converted into useful robot motion coordinates.

---

## Windows / WSL Integration

In some setups, the camera may run on Windows while the robot is operated using ROS inside WSL. In this case, you must implement a simple communication method between the two systems.

You may choose any practical approach, for example:

* TCP socket communication
* UDP communication
* HTTP request/API
* Shared file
* ROS bridge or custom interface
* Manual test interface for early debugging

The goal is to pass the QR detection information from the camera system to the robot control system.

Your report should clearly explain the communication method used and how the robot receives the cube position and angle.

---

## Safety Requirements

Safety is a major part of this project. The robot must never be operated carelessly or at high speed during development.

You must follow these safety rules:

1. Emergency stop button must be available and tested before running the robot.
2. Start with very slow robot speed.
3. Monitor the robot workspace at all times.
4. Do not assume the code will behave correctly on the first run.
5. Keep hands, tools, and objects away from the robot while it is moving.
6. Test each part separately before running the full sequence.
7. Use small motion steps when testing robot movement.
8. Stop immediately if the robot motion is unexpected.

Before running the full pick-and-place task, you should first test:

* QR code detection only.
* Robot motion without the cube.
* Robot movement to a safe point above the cube.
* Gripper opening and closing.
* Full pick-and-place at reduced speed.

---

## Physical Integration Requirements

You must physically integrate the camera into the setup. The camera should be stable and should not move during operation, otherwise calibration and position estimation will become unreliable.

You should also make sure that:

* The QR code is clearly visible.
* Lighting is sufficient.
* The cube does not reflect too much light.
* The camera view covers the full working area.
* The robot does not collide with the camera or its mount.
* Cables are safely routed away from the robot motion area.

At the end of the project, you must remove the camera and return the robot workspace to its original condition.

---

## Deliverables

You must submit:

1. A short project report.
2. Photos of the physical setup.
3. QR code detection code.
4. Robot control code.
5. A short video showing the system working.
6. Explanation of the camera-to-robot coordinate conversion.
7. Safety procedure followed during testing.
8. Final results and limitations.

---


## Notes

You are free to choose your own implementation method, camera type, programming language, and communication approach. However, the final system must demonstrate a real physical integration between the camera, QR code detection, and robot motion.

The project does not need to be perfect, but it must be safe, functional, and clearly explained.
