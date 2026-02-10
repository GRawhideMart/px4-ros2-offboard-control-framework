# PX4 + ROS 2 Offboard Control Framework

An engineering-grade framework for **PX4 Offboard Control** using **ROS 2 Humble**. 
This project implements a high-performance trajectory tracking node utilizing **Micro-XRCE-DDS** for low-latency communication and **Velocity Feed-Forward** logic to minimize tracking error during dynamic maneuvers.

## 🏗 System Architecture

The system is designed to bridge high-level path planning (ROS 2) with low-level flight control (PX4) via the uORB middleware.

* **Middleware:** Micro-XRCE-DDS (UDPv4)
* **Control Strategy:** Position Control + Velocity Feed-Forward (to eliminate phase lag)
* **Simulation:** Gazebo (Garden/Harmonic) or Classic
* **Visualization:** RViz2 & PlotJuggler (Pre-configured)

## 🚀 Installation & Setup

This repository uses **Git Submodules** to manage dependencies like `PX4-Autopilot` and `px4_msgs`.

### 1. Clone the Repository
You **must** clone with the `--recursive` flag to automatically download the required PX4 submodules.

```bash
git clone --recursive [https://github.com/GRawhideMart/px4_offboard_tracking.git](https://github.com/GRawhideMart/px4_offboard_tracking.git)
cd px4_offboard_tracking

```

*If you forgot to clone recursively, run:* `git submodule update --init --recursive`

### 2. Build the ROS 2 Workspace

Ensure you have a working ROS 2 Humble environment.

```bash
cd ros2_ws

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build the workspace (including px4_msgs and the control node)
colcon build

# Source the overlay
source install/setup.bash

```

### 3. Build & Run PX4 SITL (Simulation)

In a separate terminal, build and launch the PX4 Software In The Loop (SITL) simulation.

```bash
cd PX4-Autopilot
make px4_sitl gz_x500

```

*Wait until the drone spawns in Gazebo.*

---

## 🚁 How to Run

Once the simulation is running, launch the entire control stack with a single command:

```bash
# In a new terminal (inside the project root)
source ros2_ws/install/setup.bash

# Launch the Agent, Micro-DDS, RViz2, and PlotJuggler
ros2 launch px4_trajectory_tracking system_integration.launch.py

```

### Expected Behavior

1. **MicroXRCE-DDS Agent** connects to the simulator on UDP port 8888.
2. The drone performs a pre-flight check and **arms automatically**.
3. The drone takes off to an altitude of **-5.0m**.
4. It begins tracking a **circular trajectory** with feed-forward velocity inputs.

---

## 🧠 Control Logic: The "Feed-Forward" Approach

A standard position controller () often suffers from steady-state error (lag) when tracking moving targets. To solve this, this node computes the analytical derivative of the trajectory:

By sending both **Position** and **Velocity** setpoints to PX4, the internal flight controller uses the velocity vector as a feed-forward term, reducing phase lag to near zero.

```python
# Simplified Logic
target_x = RADIUS * cos(omega * t)
target_vx = -RADIUS * omega * sin(omega * t)  # Feed-Forward Term

msg.position = [target_x, target_y, altitude]
msg.velocity = [target_vx, target_vy, 0.0]

```

## 🛠 Dependencies

* **ROS 2 Humble** (Desktop Full recommended)
* **PX4 Autopilot** (v1.14+)
* **Micro-XRCE-DDS Agent**
* **Python 3.10+** (rclpy, numpy)

## 📄 License

This project is open-source and available under the MIT License.