# PX4_Offboard_Simulation
---
## 🧑‍💻 Author
**T.Jaswanth Reddy**    
• [Gmail](reddyjaswanth525@gmail.com)**|** 
• [LinkedIn])(https://www.linkedin.com/in/thugu-jaswanth-reddy-12a72828b/)**|**
• [Git](https://github.com/JaswanthReddyT)


# Offboard Drone Waypoint Navigation (ROS 2 + PX4 + Gazebo Classic)

This project demonstrates **autonomous waypoint navigation** in offboard mode** using a PX4-based drone simulation (Iris model) in **Gazebo Classic**.  
An offboard ROS 2 node (`offboard_waypoints.py`) sends position setpoints (waypoints) to the **Pixhawk flight controller (PX4 SITL)** through **MAVROS**, guiding the drone through **9 waypoints (W0–W8)** and finally returning to launch after mission completion.

---

## 🧠 Project Objective

- Simulate a drone flying in **offboard mode** using **ROS 2**, **PX4**, and **Gazebo Classic**.  
- Send **9 waypoints (W0–W8)** from an offboard computer to PX4 via MAVROS.  
- Execute a full mission flow:
  1. **W0** – Arm and take off vertically.  
  2. **W1–W8** – Navigate through all waypoints sequentially.  
  3. **Return to Launch (RTL)** – Automatically land after W8.

---

## ⚙️ System Requirements

| Component | Version / Notes |
|------------|----------------|
| **ROS 2** | Humble |
| **PX4 Autopilot** | v1.13+ (SITL mode) |
| **Gazebo Classic** | Included with PX4 (Iris model) |
| **MAVROS** | Installed and configured for PX4 |
| **QGroundControl (Optional)** | For real-time monitoring |
| **Python 3** | Required for `rclpy`, `mavros`, and `matplotlib` |

---

## 🧰 Install Required Dependencies

If not already installed:
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-gazebo-ros-pkgs
sudo apt install python3-colcon-common-extensions python3-matplotlib
📦 Creating the ROS 2 Python Package
If you don’t already have a workspace, create one first:

bash
Copy code
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create a new ROS 2 Python package
ros2 pkg create --build-type ament_python offboard_control --dependencies rclpy mavros_msgs geometry_msgs sensor_msgs visualization_msgs std_msgs
This will generate a folder structure like:

arduino
Copy code
ros2_ws/
├── src/
│   └── offboard_control/
│       ├── package.xml
│       ├── setup.py
│       └── offboard_control/
│           └── __init__.py
🧭 Clone the Repository into the Package
Now clone this GitHub repository into your src directory:

bash
Copy code
cd ~/ros2_ws/src/offboard_control/
git clone https://github.com/<your-username>/<repo-name>.git
Move the offboard_waypoints.py file into the offboard_control/ folder inside the package and ensure it’s executable:

bash
Copy code
mv <repo-name>/offboard_waypoints.py ./offboard_control/
chmod +x ./offboard_control/offboard_waypoints.py
🧱 Build the Package
From the root of your workspace:

bash
Copy code
cd ~/ros2_ws
colcon build
After building, source your workspace:

bash
Copy code
source install/setup.bash
🚀 Step-by-Step Guide to Perform the Task
🧭 Step 1: Launch the Drone in Gazebo (PX4 SITL)
In a new terminal:

bash
Copy code
cd ~/PX4-Autopilot
make px4_sitl gazebo
✅ This launches PX4 SITL and opens Gazebo Classic with the Iris quadcopter model.
You should see the drone appear in the simulation world.

⚡ Step 2: Launch the MAVROS Node
In another terminal:

bash
Copy code
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@
You should see output similar to:

css
Copy code
[ INFO] [mavros]: MAVROS started. Connected to PX4.
This confirms the MAVROS bridge is connected to PX4.

🧠 Step 3: Run the Offboard Node (Python File)
In a third terminal:

bash
Copy code
source ~/ros2_ws/install/setup.bash
ros2 run offboard_control offboard_waypoints
or (if running directly):

bash
Copy code
cd ~/ros2_ws/src/offboard_control/offboard_control/
python3 offboard_waypoints.py
The node will:

Arm the drone (W0)

Take off vertically

Fly through W1–W8

Return to launch (RTL) and land automatically

🛰️ Step 4: Monitor Using QGroundControl

Launch QGroundControl you can the drone tavelling through the given waypoints in the map.


# 🚀 Enhanced Offboard Waypoint Navigation

This project implements a robust, autonomous navigation system for an Iris drone using **ROS 2** and **PX4/MAVROS** to execute a rectangular waypoint mission in **Offboard mode**. The system includes advanced fail-safes, adaptive navigation logic for environmental disturbances (like gusty wind), and comprehensive logging.

## ✨ Project Overview

The primary task is to command the drone to follow a pre-defined set of **9 waypoints (WP0 to WP8)** in a rectangular pattern.

1.  **WP0 (Start/Home):** The drone arms and takes off vertically to the mission altitude.
2.  **WP1 to WP8:** The drone navigates through the waypoints to complete the rectangular circuit.
3.  **Mission End:** After reaching **WP8**, the drone executes a Return to Launch (`AUTO.RTL`).
4.  **Laps:** The mission is configured to fly for a specified number of laps (default is 1 full circuit).

### Key Features Implemented

* [cite_start]**Robust Waypoint Handling:** Maximum of **2 attempts** per waypoint with configurable timeout before skipping[cite: 133].
* [cite_start]**Adaptive Approach:** If the position error exceeds **1.9 meters**, the drone performs a corrective "approach-from-better-angle" maneuver before re-attempting the waypoint[cite: 134, 191].
* [cite_start]**Enhanced Communication Loss Handling[cite: 135, 217]:**
    * [cite_start]**Case 1:** If the mission is progressed (reached $\ge 1$ waypoint), the drone attempts to finish the mission locally[cite: 136, 220].
    * [cite_start]**Case 2:** If the mission has not progressed significantly (reached $\le 3$ waypoints), it immediately commands Return to Launch (`AUTO.RTL`)[cite: 137, 219].
* [cite_start]**Comprehensive Logging:** Outputs include a CSV log, and plots for XY tracking, error over time, and waypoint status[cite: 195, 201, 202, 203, 204].

***

## ⚙️ Prerequisites and Installation

This project assumes the following core systems are already installed and configured on your machine.

* **Operating System:** Ubuntu (ROS 2 supported)
* **ROS 2 Distribution:** Humble/Humble [cite: 123]
* [cite_start]**Autopilot Software:** PX4 v1.15 (SITL setup) [cite: 124]
* [cite_start]**Simulator:** Gazebo Classic (Iris model) [cite: 125]
* **MAVROS:** Installed and configured as the communication bridge [cite: 126]
* **Python Dependencies:** `rclpy`, `matplotlib` (optional, for plotting).

### 1. Clone and Setup

Clone this repository into your ROS 2 workspace's `src` folder (e.g., `~/ros2_ws/src`).

```bash
cd ~/ros2_ws/src
git clone <YOUR_REPO_URL>


