# LD90 ARCL Bridge (ROS 2 Humble)

This package provides a lightweight **ROS 2 interface bridge** for the **Omron LD90 AMR**, enabling integration between the robot’s **ARCL (Advanced Robotics Command Language)** server and the ROS 2 ecosystem.  
It also includes a **simulated ARCL mode** and **RViz visualization** for development and testing without a physical robot.

---

## 🧠 Features

- **ARCL TCP client** that connects to the LD90 AMR’s ARCL server (`port 7260` by default)
- **Simulated ARCL mode** for testing (`fake_arcl_server.py`)
- **ROS 2 topics** for velocity command, odometry, and robot state publishing
- **RViz visualization** using a minimal URDF model
- **Launch files** for both simulation and real robot connections
- Designed to work **inside WSL 2 (Windows 11)** with **CycloneDDS / FastDDS**

---

## 🧩 Package structure

```
ld90_arcl_bridge/
├── config/
│   └── bridge_config.yaml        # Bridge configuration (host, port, frame names)
├── launch/
│   ├── bridge_sim.launch.py      # Uses fake ARCL server for testing
│   └── bridge_real.launch.py     # Connects to actual LD90 over network
├── ld90_arcl_bridge/
│   ├── bridge_node.py            # Main ROS 2 Node (TCP ARCL Bridge)
│   ├── fake_arcl_server.py       # Fake ARCL for simulation
│   └── __init__.py
├── rviz/
│   └── ld90_bridge.rviz          # Saved RViz layout
├── urdf/
│   └── ld90_base.urdf            # Simple URDF model of LD90
├── package.xml
├── setup.py
└── setup.cfg
```

---

## 🚀 How to Run (Simulation Mode)

1. **Build the package**
   ```bash
   cd ~/ws_ros2
   colcon build --packages-select ld90_arcl_bridge
   source install/setup.bash
   ```

2. **Run the simulated bridge**
   ```bash
   ros2 launch ld90_arcl_bridge bridge_sim.launch.py
   ```

   This will:
   - Start the fake ARCL server  
   - Launch the bridge node  
   - Spawn a simple robot model in RViz

3. You should see:
   - `/ld90_bridge`, `/robot_state_publisher`, and `/rviz2` nodes  
   - Topics such as `/cmd_vel`, `/odom`, `/tf`, `/tf_static`  
   - The LD90 model moving in a circle (fake odometry motion)

---

## 🤖 How to Connect to the Real LD90

1. Edit `config/bridge_config.yaml`:
   ```yaml
   arcl_host: "192.168.0.xxx"   # LD90 IP
   arcl_port: 7260
   frame_id: "odom"
   child_frame_id: "base_link"
   ```

2. Launch:
   ```bash
   ros2 launch ld90_arcl_bridge bridge_real.launch.py
   ```

3. You can now:
   - Send velocity commands on `/cmd_vel`
   - View odometry and TF in RViz

---

## 🧩 Environment Variables (WSL2 Recommended)

For stable DDS communication inside WSL 2:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=$HOME/.CycloneDDS.xml
```

Example `.CycloneDDS.xml`:
```xml
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <EnableMulticastLoopback>false</EnableMulticastLoopback>
    </General>
  </Domain>
</CycloneDDS>
```

---

## 🧪 Testing Communication

In one terminal:
```bash
ros2 run demo_nodes_cpp talker
```

In another:
```bash
ros2 run demo_nodes_cpp listener
```

If both nodes communicate, DDS is configured correctly.

---

## 🗂 Future Extensions

- Add **Nav2 integration** (AMCL + map server)
- Publish **laser scan / pose** data from ARCL messages
- Add **MoveIt 2 interface** for docking actions
- Replace fake odometry with real `/odom` data from LD90

---

## 👤 Author

**Win Tun Latt**  
Singapore Polytechnic — Robotics & Automation  
📧 wintunlatt@example.com  
🔗 [https://github.com/wintunlatt](https://github.com/wintunlatt)
