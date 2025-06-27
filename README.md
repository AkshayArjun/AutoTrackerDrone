# AutoTrackerDrone

**AutoTrackerDrone** is a ROS 2-based project integrating a custom control stack with object tracking to enable autonomous drone behavior. The system uses **PX4 SITL** and supports advanced control architectures such as **Feedback Linearisation (FBL)** and **LPV-MPC**, while laying the groundwork for future path planning via FOAM.

---

## 🧠 Key Features

- 🧩 Modular ROS 2 architecture with custom-built controller layers
- 🎯 Feedback Linearisation + LPV-MPC controller stack
- 🔍 Planned integration of visual and LiDAR-based tracking
- 📍 Works with PX4 SITL in Gazebo/Ignition simulation
- 🧪 Designed for rapid experimentation and controller development

---

## 📂 Repository Structure

```text
AutoTrackerDrone/
├── controller_foam/            # Main package with control + sensor logic
│   ├── sensor_interface.py     # Reads PX4 odometry
│   ├── fbl_controller.py       # Feedback linearisation
│   ├── mpc_controller.py       # LPV-MPC logic
│   ├── trajectory_generator.py # Reference trajectory source
│   └── main_controller.py      # Combined control loop (ROS 2 Node)
├── model/                      # SDF models for drone and world
├── launch/                     # Launch files (optional)
└── px4_frame_converter.py      # Coordinate/frame convention translator
```

---

## 🛠 Requirements

- ROS 2 (tested on **Humble**)
- PX4 Autopilot (via SITL)
- Gazebo / Ignition simulator
- Python 3.8+
- Dependencies:
  ```bash
  pip install numpy scipy opencv-python
  ```

---

## 🚀 Quick Start

1. Clone and build workspace:
   ```bash
   git clone https://github.com/AkshayArjun/AutoTrackerDrone.git
   cd AutoTrackerDrone
   colcon build
   source install/setup.bash
   ```

2. Run the main controller:
  (to be implemented)

3. Launch PX4 SITL and simulator separately using PX4 or a custom launch file.

---

## 🧭 Control Architecture

```
Reference Trajectory
        ↓
   FBL Controller  ←←←←←←←←←←←←←←←←←←←←
        ↓                             ↑
   LPV-MPC Controller                 ↑ Sensor Feedback
        ↓                             ↑
   Rotor Speeds (ω₁–ω₄)             Odometry (NED)
        ↓                             ↑
      PX4 SITL ←←←←←←←←←←←←←←←←←←←←←←←
```

---

## ✅ Project Status

- [x] FBL controller implemented and tested
- [x] LPV-MPC controller
- [ ] Control Architecture test with PX4 (pending)
- [ ] Path planning using FOAM (pending)
- [ ] Vision module integration (pending)
- [ ] LiDAR sensor interface (pending)
- [ ] Real-world testing (pending)

---

## 📌 Notes

- The system assumes **standard (+Z up) frame internally**, and converts inputs/outputs from/to PX4’s **NED frame** using `px4_frame_converter.py`.
- Visual tracking and LiDAR modules are to be added later under `vision_module.py`.

---

## 📜 License

MIT License

---

## 🤝 Contributions

Contributions and suggestions are welcome! Please open an issue or submit a pull request.
