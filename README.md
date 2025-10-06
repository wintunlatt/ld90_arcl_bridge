# LD90 ARCL Bridge

ROS 2 (Humble) bridge for Omron LD-90 AMR using ARCL protocol.

Features:
- Connects to ARCL server (real or simulated)
- Publishes `/odom`, `/ld90/status`, and listens to `/cmd_vel`
- Includes RViz visualization (`ld90_bridge.rviz`)
- Simple URDF model of the LD-90 base

## Run (simulation)
```bash
ros2 launch ld90_arcl_bridge bridge_sim.launch.py

## Run (real robot)
ros2 launch ld90_arcl_bridge bridge_real.launch.py

Developed and tested under WSL2 Ubuntu 22.04 + ROS 2 Humble.
