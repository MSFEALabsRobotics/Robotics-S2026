# Install WSL2 + ROS 2 and Verify It Works (Minimal)

Goal: **Windows → WSL2 Ubuntu → ROS 2 Jazzy → quick test**.

---

## 1) Install WSL2 + Ubuntu (Command Prompt)

1) Open **Command Prompt as Administrator**  
Start → type `cmd` → right‑click **Command Prompt** → **Run as administrator**

2) Install WSL + default Ubuntu:
```bat
wsl --install
```

3) Restart Windows if prompted.



5) Open Ubuntu:
```bat
wsl
```
(or open **Ubuntu** from the Start menu and create your Linux username/password)

---

## 2) Update Ubuntu

Inside the Ubuntu terminal:
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release
```

---

## 3) Install ROS 2 Jazzy (Ubuntu 24.04)

> Jazzy deb packages are for **Ubuntu 24.04 (Noble)**.

Enable Universe:
```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

Add ROS 2 key + repo:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2 Desktop:
```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

Auto-source ROS in every terminal:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4) Test ROS 2 (Required)

### 4.1 Check the CLI
```bash
ros2 --help
```

### 4.2 Talker/Listener demo (2 terminals)

Open **two Ubuntu terminals**.

**Terminal 1**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener
```

✅ If it’s working, Terminal 2 prints messages like “Hello World …”.

---

