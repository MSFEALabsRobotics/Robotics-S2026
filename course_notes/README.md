# MECH 641 / EECE 661 — Robotics (Spring 2026)
## Introduction to Robotics 🤖

Welcome to **Robotics**!  
This course is about how we make machines **move**, **sense**, **plan**, and **act intelligently** in the real world.

---

## 👤 Instructor & Course Info

**Instructor:** Hussein Hussein  
**Office:** Bechtel 503  
**Email:** hh224@aub.edu.lb  
**Office Hours:** TR 13:00–15:00 (by appointment)

### 📌 Class & Lab Schedule
- **Lectures:** 2 sessions/week (1h 15m each)  
  - **TR 15:30–16:45** — Bechtel 208
- **Lab:** 1 session/week (3h)  
  - **TR 17:00–19:45** — IOEC 402
- **Pre-/Co-requisite:** Control Systems

---

## 1) What is a Robot? (And why do we care?)

### ✅ A simple definition
A **robot** is a machine capable of carrying out a complex series of actions automatically.

But more importantly…

### ⭐ What makes a robot “robotic”?
A robot is not just a machine — it is a system that can:
- **Sense** the world (cameras, IMU, force sensors, encoders)
- **Think/Decide** (control, planning, AI)
- **Act** (motors, actuators, wheels, legs, arms)
- **Adapt** to changes (uncertainty, disturbances, real environments)

### 🧠 What is Robotics?
**Robotics** is the interdisciplinary field that combines:
- mechanical design
- electronics
- control systems
- computer science
- perception and AI
to build machines that can interact with the real world.

---

## 2) Why Robots? (The real motivation)

Robots exist because they can do tasks:
- **better**
- **faster**
- **safer**
- and sometimes **impossible** for humans

### The classic motivation: the “3 Ds”
Robots are excellent at jobs that are:
- **Dirty** 🧼 (hazardous environments, contamination)
- **Dull** 😴 (repetitive and boring tasks)
- **Dangerous** ⚠️ (high risk to human life)

### Robots improve performance in industry by increasing:
- productivity
- quality and repeatability
- speed of delivery
- cost efficiency

---

## 3) Where do we find robots today?

Robots are everywhere — not only in factories.

Examples:
- **Industrial automation** (assembly lines, welding, packaging)
- **Healthcare** (surgical robots, rehabilitation devices)
- **Service robots** (cleaning, hotel assistance)
- **Logistics** (warehouse robots, autonomous delivery)
- **Exploration** (space rovers, underwater robots)
- **Entertainment & education** (humanoids, toy robots)
- **Modern research** (legged robots, modular robots, swarm robotics)

---

## 4) A (Very) Short History of Robotics ⏳

Robotics is not “new” — it is the result of centuries of engineering.

### Key milestones
- **Ancient foundations:** simple machines like pulleys and levers enabled early automation ideas.
- **1921:** the word **“robot”** became popular from the play  
  *R.U.R. (Rossum’s Universal Robots)* by Karel Čapek.
- **1940s–1950s:** robotics started to accelerate with electronics and computers.
- **1954:** first patent for an industrial robot.
- **1961:** first industrial robot installed in a General Motors factory.
- **1970s–1980s:** major advances in sensing and control.
- **1977:** first robot with **six axes (6-DoF)** developed (Kawasaki Heavy Industries).
- **Late 1980s:** mobile robots emerged (initially in manufacturing and logistics).
- **1990s:** robots entered advanced medical applications (robotic surgery).
- **21st century:** robotics expanded massively due to:
  - AI and machine learning
  - cheaper sensors
  - stronger computing power
  - open-source software (ROS)

---

## 5) Types of Robots (A quick map of the field)

Robots come in many shapes and capabilities. Common categories include:

### 1. Industrial Robots 🏭
- high precision, high speed
- repetitive tasks (manufacturing, assembly)
- often large and expensive

### 2. Service Robots 🏨
- interact with people and provide services
- hospitals, hotels, airports
- cleaning, security, assistance

### 3. Mobile Robots 🚗
- move in their environment
- ground / aerial / underwater
- autonomous or teleoperated
- used in logistics, agriculture, mining, exploration

### 4. Medical Robots 🏥
- surgical assistance
- rehabilitation and therapy
- monitoring and precision procedures

### 5. Social Robots 🙂🤝
- human interaction
- companionship, education, entertainment

### 6. Drones ✈️
- delivery, inspection, mapping, surveillance
- fast deployment and large coverage

### 7. Soft Robots 🧸
- flexible materials
- safe interaction with humans
- useful in delicate environments

---

## 6) The Future of Robotics 🚀

Robotics is one of the most impactful technologies of this century.

### Major drivers
#### 🧠 Artificial Intelligence (AI)
AI enables robots to:
- interpret images and sensor signals
- understand environments
- make decisions
- improve with experience

#### 🛰️ Autonomy
Autonomous robots will reshape:
- transportation (self-driving vehicles)
- logistics and delivery
- drones and aerial systems

#### 🩺 Healthcare
Robotics will improve:
- precision surgery
- rehabilitation technologies
- quality and accessibility of care

### Big picture
The future of robotics is promising because it can improve:
- efficiency
- safety
- productivity
- quality of life

---

## 7) What this course is really about 🎯

This course focuses mainly on **robot arms (serial manipulators)** and how we make them perform tasks reliably. The theory extends for other types of robots, such as paralle manipulators and legged robots. The lab deals with many types of robots.

Think of this course as answering:

> “How can we mathematically model a robot, plan its motion, and control it to do something useful? How can we do this in practice?”

### Core pillars

#### A) Robot Modeling
##### **Kinematics**
Kinematics describes **motion without forces**.
- With link geometry + joint types, we can compute:
  - position
  - orientation
  - velocities

##### **Dynamics**
Dynamics describes **motion under forces/torques**.
- With mass and inertia + external forces, we can predict:
  - required joint torques
  - accelerations and motion response

---

#### B) Planning
##### **Motion Planning**
Finding a collision-free path from start to goal.

##### **Trajectory Generation**
Motion planning + timing:
- “Where to go?” + “When to be there?”

---

#### C) Control
Control is how we make the robot **follow the plan** in the real world.
Examples:
- joint angle tracking
- end-effector trajectory tracking
- disturbance rejection (noise, friction, payload)

---

## 8) Lab Experience 🧪

The lab is where robotics becomes real.

You will practice:
- ROS 2 on Ubuntu
- building ROS nodes and applications
- using robotics tools (RViz, Gazebo)
- integrating algorithms with real robotic systems

### Platforms and projects may include
- robotic arm experiments
- Unitree Go1 / Go2 applications
- teamwork-based robotics tasks

> **Goal:** You don’t just learn robotics — you *build robotics*.

---

## 9) Course Roadmap (High-level)

Topics include:
- Introduction to Robotics
- Rigid Motions
- Forward Kinematics
- Jacobians
- Inverse Kinematics
- Dynamics
- Motion Planning & Trajectory Generation

---

## 10) Grading (Quick view)

- Team quizzes and activities: **10%**
- Midterm: **20%**
- Lab project: **25%**
- Final: **45%**

Expected coverage:
- **Midterm:** Chapters 2, 3, 4
- **Final:** DH + Chapters 4, 5, 6, 7

---

## 📚 Reference Textbook

M.W. Spong, S. Hutchinson, M. Vidyasagar  
**Robot Modeling and Control**, 2nd Edition, Wiley, 2018

---

## ✅ Takeaway Summary (1 minute review)

By the end of this lecture, you should remember:
- Robotics combines **mechanics + control + computing + intelligence**
- Robots are ideal for the **3Ds**: Dirty, Dull, Dangerous
- Robotics evolved from early automation to modern AI-driven systems
- The lab will help you turn theory into working systems
