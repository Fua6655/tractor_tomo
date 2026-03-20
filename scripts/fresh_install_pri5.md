# 🚜 TOMO Tractor – Full Setup Guide (Raspberry Pi 5 + ROS2 Jazzy)

This guide describes a **complete clean setup from scratch** for Raspberry Pi 5, including:

* Ubuntu Desktop
* ROS2 Jazzy
* PS4 controller (Bluetooth)
* Web UI (FastAPI)
* Full TOMO system launch

---

# 🧰 1. OS Installation

## ✅ Recommended:

* **Ubuntu Desktop 24.04 (Raspberry Pi)**

📌 Install using:

* Raspberry Pi Imager → Ubuntu Desktop 24.04

---

# 🔐 2. Enable SSH (Remote Access)

```bash
sudo apt update
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh
```

Check IP address:

```bash
hostname -I
```

---

# 🎮 3. PS4 Controller (Bluetooth)

## Pairing:

1. Open **Settings → Bluetooth**
2. Hold **PS + SHARE** (enter pairing mode)
3. Connect to **Wireless Controller**

## Test:

```bash
ls /dev/input/js*
jstest /dev/input/js0
```

Expected:

```
/dev/input/js0
```

---

# 🧠 FIX: Permissions (REQUIRED)

```bash
sudo usermod -aG input $USER
sudo reboot
```

---

# 🤖 4. Install ROS2 Jazzy (FULL)

```bash
sudo apt update
sudo apt install curl gnupg lsb-release -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install ros-jazzy-desktop -y
```

---

# 🌍 5. Environment Setup

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 🧰 6. Development Tools

```bash
sudo apt install \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  build-essential \
  git \
  -y
```

---

# 🔗 7. rosdep Initialization

```bash
sudo rosdep init
rosdep update
```

---

# 📦 8. Clone Project (develop branch)

```bash
cd ~
git clone -b develop https://github.com/Fua6655/tractor_tomo.git
```

---

# 📥 9. Install Dependencies

```bash
cd ~/tractor_tomo/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

# 🔨 10. Build Workspace

```bash
colcon build --symlink-install
```

---

# 🌍 11. Source Workspace

```bash
echo "source ~/tractor_tomo/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 🌐 12. Install Web Dependencies (FastAPI)

```bash
python3 -m pip install fastapi uvicorn[standard] --break-system-packages
```

---

# 🚜 13. Launch System

```bash
cd ~/tractor_tomo
./scripts/tomo_launch.sh
```

---

# 🌐 14. Web UI Access

Open in browser:

```
http://<RASPBERRY_IP>:8000
```

---

# 🎮 15. Test Controller → ROS

```bash
ros2 topic echo /joy
```

Expected output:

```
axes: [...]
buttons: [...]
```

---

# 🎯 FINAL STATE

After completing all steps, you should have:

* ✅ ROS2 Jazzy FULL
* ✅ PS4 controller (Bluetooth, auto-reconnect)
* ✅ Web UI (FastAPI + WebSocket)
* ✅ Full TOMO launch system
* ✅ SSH remote access

---

# ⚠️ IMPORTANT NOTES

* ❌ DO NOT use Python virtual environments (venv)
* ❌ DO NOT create additional ROS workspaces
* ✔ Use the existing `tractor_tomo/ros2_ws`
* ✔ Use system Python + rosdep

---

# 🧠 Tested On

* Raspberry Pi 5
* Ubuntu 24.04 Desktop
* ROS2 Jazzy

---