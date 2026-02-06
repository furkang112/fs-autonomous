# System Architecture
This document describes the internal structure of the autonomous system.
---
## Perception
Responsible for detecting cones and extracting relevant features.
Inputs:
- Camera images
- LiDAR point clouds
Outputs:
- Cone positions
- Track boundaries
---
## Localization
Estimates vehicle position and orientation.
Inputs:
- IMU
- Wheel speed
- Perception output
Outputs:
- Vehicle pose (x, y, yaw)
---
## Planning
Generates a feasible path and trajectory.
Inputs:
- Vehicle pose
- Track information
Outputs:
- Target path
- Target speed
---
## Control
Converts trajectory into steering and throttle commands.
Inputs:
- Planned trajectory
- Vehicle state
Outputs:
- Steering angle
- Throttle / brake
kanka bunun içinde güzel bir şey hazırlar mısın
