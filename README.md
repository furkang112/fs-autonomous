# 🏎️Formula Student Autonomous Systems

This repository contains the autonomous driving software developed
by the team for Formula Student Driverless / Autonomous competitions.

The project focuses on building a modular, scalable and testable
autonomous system for a Formula Student vehicle.

---

## System Overview

The autonomous system is designed with a modular architecture:

- **Perception**: Detects cones and environment using sensors
- **Localization**: Estimates vehicle position and orientation
- **Planning**: Generates path and trajectory
- **Control**: Controls steering and speed
- **Simulation**: Testing and validation in simulation

## 📁 Repository Structure

```
fs-autonomous/
│
├── 📁 docs/
│ ├── 📄 system_overview.md
│ ├── 📄 architecture.md
│ └── 📄 competition_rules.md
│
├── 📁 perception/
│ ├── 📁 camera/
│ ├── 📁 lidar/
│ └── 📁 cone_detection/
│
├── 📁 localization/
│ ├── 📁 ekf/
│ ├── 📁 slam/
│ └── 📁 sensor_fusion/
│
├── 📁 planning/
│ ├── 📁 path_planning/
│ └── 📁 trajectory/
│
├── 📁 control/
│ ├── 📁 steering/
│ ├── 📁 speed/
│ └── 📁 controllers/
│
├── 📁 simulation/
│ ├── 📁 gazebo/
│ └── 📁 rviz/
│
├── 📁 utils/
│ └── 📄 helpers.py
│
├── 📁 tests/
│
├── 📄 README.md
├── 📄 requirements.txt
└── 📄 .gitignore
```

## Technologies

- **Programming Languages**: Python, C++
- **Middleware**: ROS 2
- **Simulation**: Gazebo, RViz
- **Libraries**: OpenCV, NumPy

---

## Development Status

🚧 The project is under active development.  
Initial focus is on simulation and basic autonomous functionality.

---

## Team

Formula Student Autonomous Team  

