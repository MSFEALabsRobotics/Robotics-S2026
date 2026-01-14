# MECH 641 / EECE 661 — Robotics (Spring 2026)
## Lecture 1 — Introduction to Robotics

**Instructor:** Hussein Hussein  
**Office:** Bechtel 503  
**Email:** hh224@aub.edu.lb  
**Office Hours:** TR 13:00–15:00 (by appointment)

---

## 1. Course Logistics (Quick Summary)

- **Lectures:** 2 sessions per week (1h 15m each)  
  - **Time:** TR 15:30–16:45  
  - **Location:** Bechtel 208
- **Lab:** 1 session per week (3h)  
  - **Time:** TR 17:00–19:45  
  - **Location:** Oxy 402
- **Pre-/Co-requisite:** Control Systems

---

## 2. What is Robotics?

### Definition (Core Idea)
- A **robot** is a machine capable of carrying out a complex series of actions automatically.
- **Robotics** is an interdisciplinary field that focuses on:
  - design
  - construction
  - operation
  - control and use of robots

### Why do we use robots?
Robots are often used to replace or assist humans in tasks that are:
- repetitive
- time-consuming
- unsafe or hazardous

Robotics enables automation to improve:
- performance capacity
- cost efficiency
- product quality
- delivery time

---

## 3. Why Robots? (The 3Ds)

Robots are especially useful for jobs that are:
- **Dirty**
- **Dull**
- **Dangerous**

Robots have been used in many areas such as:
- industrial automation and manufacturing
- service applications
- medicine and healthcare
- assistance and support tasks
- entertainment

---

## 4. A Very Brief History of Robotics

### Key milestones
- **1921:** The word “robot” became popular through the play *R.U.R. (Rossum’s Universal Robots)*.
- **Ancient foundations:** early machines such as pulleys and levers enabled mechanical automation ideas.
- **1940s–1950s:** major development started with electronics and computers.
- **1954:** first patent for an industrial robot.
- **1961:** first industrial robot installed in a General Motors factory.
- **1970s–1980s:** rapid progress in sensors and control systems.
- **1977:** first 6-axis industrial robot developed (Kawasaki Heavy Industries).
- **Late 1980s:** emergence of mobile robots (initially in factories/logistics).
- **1990s:** advanced applications (example: robotic surgery).
- **21st century:** growth accelerated with AI and machine learning, and robots expanded into daily life.

---

## 5. Types of Robots (Common Categories)

1. **Industrial Robots**
   - high precision and speed
   - repetitive tasks in factories and assembly lines

2. **Service Robots**
   - interact with people
   - hospitals, hotels, airports
   - cleaning, security, assistance

3. **Mobile Robots**
   - move in the environment (ground / air / water)
   - logistics, agriculture, mining, space exploration
   - autonomous or teleoperated

4. **Medical Robots**
   - surgery assistance, therapy, rehabilitation
   - patient monitoring and precision procedures

5. **Social Robots**
   - companionship, education, interaction

6. **Drones**
   - delivery, surveillance, mapping, inspection

7. **Soft Robots**
   - flexible materials
   - safe interaction and delicate environments

---

## 6. The Future of Robotics (Big Picture)

Robotics is expected to grow rapidly due to:
- **Artificial Intelligence (AI)**
  - perception, decision-making, and autonomy
- **Autonomous systems**
  - self-driving vehicles and drones
- **Healthcare robotics**
  - improved quality of care and accessibility

Overall impact:
- improved efficiency
- improved safety
- improved quality of life

---

## 7. What This Course Focuses On

This course mainly targets **robot arms (serial manipulators)** and how to make them perform tasks.

### Main pillars
#### Robot Modeling
- **Kinematics**
  - motion description without forces
  - geometry + joints → position/orientation
- **Dynamics**
  - motion under forces and torques
  - requires mass and inertia properties

#### Planning
- **Motion planning:** finding a path
- **Trajectory generation:** path + timing/schedule

#### Control
- tracking a desired trajectory
- joint-level motor control

---

## 8. Lab Overview (What You Will Practice)

Lab sessions may include:
- ROS 2 on Ubuntu
- building ROS applications and nodes
- using ROS tools and packages (RViz, Gazebo)
- advanced projects on:
  - robotic arm platform
  - Unitree Go1 / Go2
- exploring LLMs for robot decision-making (high-level concept)

---

## 9. Course Outline (High-Level)

Topics include:
- Introduction to Robotics
- Rigid Motions
- Forward Kinematics
- Jacobians
- Inverse Kinematics
- Dynamics
- Motion Planning & Trajectory Generation

---

## 10. Grading (Summary)

- Team quizzes and activities: **10%**
- Midterm: **20%**
- Lab project: **25%**
- Final: **45%**

Midterm coverage (expected):
- Chapters 2, 3, 4

Final coverage (expected):
- DH + Chapters 4, 5, 6

---

## Suggested Reading
**Spong, Hutchinson, Vidyasagar**  
*Robot Modeling and Control*, 2nd Edition, Wiley, 2018
