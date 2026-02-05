<img width="351" height="223" alt="image" src="https://github.com/user-attachments/assets/37c0a937-682e-456f-a26b-25309897611f" />


<img width="822" height="476" alt="image" src="https://github.com/user-attachments/assets/b19b2208-8094-4a30-9c31-33d5b5f34b57" />


# CycloneDDS + Unitree SDK2 (Go2) — Quickstart Tutorial

This guide installs **CycloneDDS**, sets environment variables, installs **Unitree SDK2 Python**, then shows:
- how to **grab a camera frame** (OpenCV)
- how to **send basic motion commands**

---

## 1) Install CycloneDDS (from source)

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

---

## 2) Set environment variables

> Put these in your `~/.bashrc` if you want them to persist across terminals.

```bash
export CYCLONEDDS_HOME=$HOME/cyclonedds/install
export CycloneDDS_DIR=$CYCLONEDDS_HOME/lib/cmake/CycloneDDS
export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH
```

---

## 3) Install Unitree SDK2 (Python)

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd ~/unitree_sdk2_python
pip3 install -e . --break-system-packages
```

---

## 4) Use the mini drivers (Python scripts)

### 4.1 Grab a camera frame (OpenCV)

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
import cv2
import numpy as np
import subprocess

# Find Linux interface name that has this IPv4 assigned (edit IP to yours)
iface = lambda ip: next(
    (l.split()[1] for l in subprocess.check_output(["ip", "-o", "-4", "addr", "show"], text=True).splitlines()
     if f"inet {ip}/" in l),
    None
)

ChannelFactoryInitialize(0, iface("192.168.123.99"))

client = VideoClient()
client.SetTimeout(3.0)
client.Init()

# Get image sample
code, data = client.GetImageSample()

# Decode to OpenCV image
image_data = np.frombuffer(bytes(data), dtype=np.uint8)
image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

cv2.imshow("front_camera", image)
cv2.waitKey(0)
```

---

### 4.2 Drive the robot (basic SportClient commands)

```python
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
import subprocess

# Find Linux interface name that has this IPv4 assigned (edit IP to yours)
iface = lambda ip: next(
    (l.split()[1] for l in subprocess.check_output(["ip", "-o", "-4", "addr", "show"], text=True).splitlines()
     if f"inet {ip}/" in l),
    None
)

ChannelFactoryInitialize(0, iface("192.168.123.99"))

sport_client = SportClient()
sport_client.SetTimeout(10.0)
sport_client.Init()

# Examples (uncomment what you need)
# sport_client.Damp()        # Damp
# sport_client.StandUp()     # Stand up
sport_client.StandDown()     # Stand down

# sport_client.Move(0.3, 0, 0.5)  # Move (vx, vy, yaw_rate) — try small values
# sport_client.StopMove()         # Stop
# sport_client.FreeWalk()         # Free-walk mode

# If looping commands, sleep a bit
# time.sleep(1)
```

---

## Notes / common tweaks
- Replace `192.168.123.99` with the IPv4 that belongs to the network interface connected to the robot.
- If `iface(...)` returns `None`, run:
  ```bash
  ip -o -4 addr show
  ```
  and confirm which interface has the IP you want to use.
