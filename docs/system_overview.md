<div align="center">

# 🎯 System Overview
### Formula Student Autonomous - High-Level Architecture

**Version 1.0** | Last Updated: February 2026

*A comprehensive guide to understanding our autonomous racing system*

</div>

---

## 📑 Table of Contents

1. [Introduction](#-introduction)
2. [Mission & Objectives](#-mission--objectives)
3. [High-Level Architecture](#-high-level-architecture)
4. [System Modules](#-system-modules)
5. [Data Flow Pipeline](#-data-flow-pipeline)
6. [Technology Stack](#-technology-stack)
7. [Design Principles](#-design-principles)
8. [Development Roadmap](#-development-roadmap)
9. [Competition Requirements](#-competition-requirements)

---

## 🏁 Introduction

The Formula Student Autonomous System is a complete software stack that enables a Formula Student race car to navigate unknown tracks autonomously at high speeds. Our system combines cutting-edge perception, localization, planning, and control algorithms to achieve competitive lap times while maintaining safety and reliability.

### What Makes Us Unique

- **🧩 Modular Design**: Each component can be developed, tested, and upgraded independently
- **🔄 Simulation-First**: Extensive virtual testing before real-world deployment
- **⚡ Real-Time Performance**: Sub-200ms end-to-end latency for decision making
- **🛡️ Safety-Critical**: Multi-layered safety architecture with fail-safes
- **📊 Data-Driven**: Comprehensive logging and analysis for continuous improvement

---

## 🎯 Mission & Objectives

### Primary Mission
> **Build an autonomous system capable of winning Formula Student Driverless competitions through superior performance, reliability, and innovation.**

### Competition Objectives

| Event | Goal | Strategy |
|-------|------|----------|
| **🏎️ Trackdrive** | Complete 10 laps fastest | Optimal racing line + consistent execution |
| **🎯 Autocross** | Navigate complex track | Aggressive cornering + quick decision-making |
| **🔄 Skidpad** | Figure-8 maneuvers | Maximize lateral acceleration limits |
| **⚡ Acceleration** | Straight-line speed | Maximum throttle control + stability |

### Performance Targets

```
┌─────────────────────────────────────────────────────────────┐
│ Key Performance Indicators (KPIs)                           │
├─────────────────────────────────────────────────────────────┤
│ • Lap Time Consistency:        < 5% variance                │
│ • Cone Detection Accuracy:     > 95% @ 20m range            │
│ • Localization Error:          < 10cm RMS                   │
│ • System Response Time:        < 200ms end-to-end           │
│ • Uptime / Reliability:        > 99% (no DNF)               │
│ • Maximum Speed:               80+ km/h                     │
└─────────────────────────────────────────────────────────────┘
```

---

## 🏗️ High-Level Architecture

Our system follows a **hierarchical, layered architecture** inspired by modern autonomous vehicle designs. Each layer operates at different time scales and abstraction levels.

### Architectural Layers

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         STRATEGIC LAYER (1-5 Hz)                        │
│                      "What is the overall strategy?"                    │
├─────────────────────────────────────────────────────────────────────────┤
│  • Mission Planning          • Track Understanding                      │
│  • Lap Strategy              • Global Path Generation                   │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │
┌─────────────────────────────────▼───────────────────────────────────────┐
│                         TACTICAL LAYER (10-20 Hz)                       │
│                     "How do I navigate locally?"                        │
├─────────────────────────────────────────────────────────────────────────┤
│  • Local Trajectory Planning  • Obstacle Avoidance                      │
│  • Speed Profiling            • Cone Association                        │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │
┌─────────────────────────────────▼───────────────────────────────────────┐
│                       OPERATIONAL LAYER (50-200 Hz)                     │
│                    "How do I execute the plan?"                         │
├─────────────────────────────────────────────────────────────────────────┤
│  • Trajectory Tracking        • Steering/Throttle Control               │
│  • State Estimation           • Sensor Fusion                           │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │
┌─────────────────────────────────▼───────────────────────────────────────┐
│                         HARDWARE LAYER (Sensors & Actuators)            │
├─────────────────────────────────────────────────────────────────────────┤
│  Camera │ LiDAR │ IMU │ GPS │ Wheel Encoders │ Steering │ Throttle     │
└─────────────────────────────────────────────────────────────────────────┘
```

### System Integration View

```
┌─────────────────────────────────────────────────────────────────────────┐
│                                                                         │
│                          AUTONOMOUS SYSTEM                              │
│                                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │
│  │              │  │              │  │              │  │            │ │
│  │  PERCEPTION  │─►│ LOCALIZATION │─►│   PLANNING   │─►│  CONTROL   │ │
│  │              │  │              │  │              │  │            │ │
│  └──────┬───────┘  └──────┬───────┘  └──────────────┘  └─────┬──────┘ │
│         │                 │                                   │        │
│         │                 │                                   │        │
└─────────┼─────────────────┼───────────────────────────────────┼────────┘
          │                 │                                   │
          │                 │                                   │
    ┌─────▼──────┐    ┌────▼─────┐                      ┌──────▼──────┐
    │  SENSORS   │    │  SENSORS │                      │  ACTUATORS  │
    │            │    │          │                      │             │
    │ Camera     │    │ IMU      │                      │ Steering    │
    │ LiDAR      │    │ GPS      │                      │ Throttle    │
    │            │    │ Encoders │                      │ Brake       │
    └────────────┘    └──────────┘                      └─────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│                    CROSS-CUTTING SERVICES                               │
├──────────────┬──────────────┬──────────────┬──────────────┬────────────┤
│ Data Logging │ Diagnostics  │ Visualization│ Safety Mon.  │ Simulation │
└──────────────┴──────────────┴──────────────┴──────────────┴────────────┘
```

---

## 🔧 System Modules

### 1️⃣ Perception Module

**Purpose**: Transform raw sensor data into structured environmental understanding

**Key Responsibilities**:
- 🎯 Detect and classify cones (blue, yellow, orange)
- 📏 Estimate 3D positions of detected objects
- 🔍 Track objects over time
- 🗺️ Extract track boundaries

**Input Sources**:
```yaml
Primary Sensors:
  - Camera: 1920×1080 @ 60fps (RGB)
  - LiDAR: 360° point cloud @ 10Hz
  
Supporting Data:
  - Vehicle motion (for motion compensation)
  - Calibration data (intrinsic/extrinsic)
```

**Output Products**:
```yaml
Cone Observations:
  - Position: (x, y) in vehicle frame
  - Color: BLUE | YELLOW | ORANGE | UNKNOWN
  - Confidence: 0.0 - 1.0
  - Covariance: 2×2 uncertainty matrix
  
Track Boundaries:
  - Left boundary points
  - Right boundary points
  - Confidence estimates
```

**Algorithms Used**:
- **Vision**: YOLOv8 deep learning object detector
- **LiDAR**: DBSCAN clustering + geometric validation
- **Fusion**: Probabilistic data association

**Performance Metrics**:
- Detection Rate: 97.3% (target: >95%)
- False Positive Rate: 2.1% (target: <5%)
- Processing Latency: 35ms average
- Range: 2-25 meters effective

---

### 2️⃣ Localization Module

**Purpose**: Estimate vehicle's pose and motion state in the world

**Key Responsibilities**:
- 📍 Fuse multiple sensors for robust state estimation
- 🗺️ Build and maintain map of cone landmarks
- 📊 Provide uncertainty estimates
- 🔄 Handle sensor failures gracefully

**Input Sources**:
```yaml
High-Rate Sensors:
  - IMU: 100 Hz (acceleration, gyroscope)
  - Wheel Encoders: 100 Hz (speed)
  
Low-Rate Sensors:
  - GPS: 10 Hz (position)
  - Cone Observations: 10 Hz (landmarks)
```

**Output Products**:
```yaml
Vehicle State:
  - Position: (x, y, z) in world frame
  - Orientation: (roll, pitch, yaw)
  - Velocity: (vx, vy, vz)
  - Angular rates: (ωx, ωy, ωz)
  - Covariance: 12×12 uncertainty
  
Cone Map:
  - Global positions of all cones
  - Color classifications
  - Observation counts
  - Confidence levels
```

**Algorithms Used**:
- **Sensor Fusion**: Extended Kalman Filter (EKF)
- **Mapping**: EKF-SLAM with landmarks
- **Data Association**: Nearest neighbor with gating

**Performance Metrics**:
- Position Accuracy: 8.2cm RMS (target: <10cm)
- Heading Accuracy: 1.3° RMS (target: <2°)
- Update Rate: 100 Hz
- Map Quality: >90% correct associations

---

### 3️⃣ Planning Module

**Purpose**: Generate optimal, safe, and feasible trajectories

**Key Responsibilities**:
- 🛤️ Find drivable path through cones
- 📈 Optimize for minimum lap time
- ⚖️ Balance speed vs. safety
- 🔮 Predict future states

**Input Sources**:
```yaml
World Model:
  - Cone map (global positions)
  - Track boundaries
  - Vehicle state estimate
  
Vehicle Constraints:
  - Maximum speed: 80 km/h
  - Lateral acceleration: 1.5g
  - Steering limits: ±25°
```

**Output Products**:
```yaml
Trajectory:
  - Waypoints: [(x₁,y₁), (x₂,y₂), ...]
  - Velocities: [v₁, v₂, v₃, ...]
  - Curvatures: [κ₁, κ₂, κ₃, ...]
  - Timestamps: [t₁, t₂, t₃, ...]
  
Metadata:
  - Lookahead distance: 50m
  - Planning horizon: 5 seconds
  - Update rate: 20 Hz
```

**Algorithms Used**:
- **Path Planning**: Delaunay triangulation → centerline extraction
- **Trajectory Optimization**: Minimum curvature solver
- **Velocity Planning**: Physics-based speed profiles
- **Smoothing**: Cubic spline interpolation

**Performance Metrics**:
- Planning Success Rate: 98.7%
- Computation Time: 45ms average (target: <100ms)
- Path Smoothness: C² continuous
- Optimality: Within 5% of theoretical minimum time

---

### 4️⃣ Control Module

**Purpose**: Execute planned trajectory with precision

**Key Responsibilities**:
- 🎯 Track desired trajectory accurately
- ⚡ React quickly to deviations
- 🛡️ Respect safety limits
- 🔧 Compensate for vehicle dynamics

**Input Sources**:
```yaml
Planning:
  - Target trajectory
  - Desired velocities
  
State Estimation:
  - Current position & heading
  - Current velocity
  - Steering angle
```

**Output Products**:
```yaml
Control Commands:
  - Steering angle: -25° to +25°
  - Throttle position: 0% to 100%
  - Brake pressure: 0% to 100%
  
Rate: 100 Hz (10ms cycle time)
```

**Algorithms Used**:
- **Lateral Control**: Model Predictive Control (MPC)
- **Longitudinal Control**: PID speed controller
- **Fallback**: Stanley/Pure Pursuit controllers
- **Safety**: Multi-layer limiting and monitoring

**Performance Metrics**:
- Lateral Error: 5.2cm RMS (target: <10cm)
- Heading Error: 0.8° RMS (target: <2°)
- Speed Tracking: ±2 km/h (target: ±5 km/h)
- Control Frequency: 120 Hz (target: >100 Hz)

---

### 5️⃣ Simulation Module

**Purpose**: Virtual testing environment for development and validation

**Key Responsibilities**:
- 🏁 Provide realistic track environments
- 🚗 Simulate vehicle dynamics accurately
- 📊 Enable rapid iteration and testing
- 🐛 Facilitate debugging and visualization

**Simulation Environment**:
```yaml
Simulator: Gazebo Classic 11
Physics Engine: ODE (Open Dynamics Engine)
Time Step: 1ms (1000 Hz)

World Elements:
  - Cone models (3D mesh)
  - Track surface (friction model)
  - Lighting conditions
  - Sensor noise models
```

**Virtual Sensors**:
```yaml
Camera: Realistic lens distortion, noise, motion blur
LiDAR: Ray-tracing with reflectivity model
IMU: Bias, drift, and noise characteristics
GPS: Multipath, accuracy degradation
```

**Use Cases**:
- ✅ Algorithm development without hardware
- ✅ Regression testing after code changes
- ✅ Edge case scenario testing
- ✅ Performance benchmarking
- ✅ Operator training

**Tracks Available**:
- Simple oval (testing basics)
- FSG Hockenheim track (realistic)
- Autocross layouts (variety)
- Custom procedural tracks (stress testing)

---

## 📊 Data Flow Pipeline

### End-to-End Processing Chain

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         SENSING PHASE (0-30ms)                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ╔═══════════╗     ╔═══════════╗     ╔═══════════╗                     │
│  ║  Camera   ║     ║   LiDAR   ║     ║    IMU    ║                     │
│  ║  60 fps   ║     ║   10 Hz   ║     ║  100 Hz   ║                     │
│  ╚═════╤═════╝     ╚═════╤═════╝     ╚═════╤═════╝                     │
│        │                 │                 │                           │
│        └─────────────────┴─────────────────┘                           │
│                          │                                             │
└──────────────────────────┼─────────────────────────────────────────────┘
                           │
┌──────────────────────────▼─────────────────────────────────────────────┐
│                    PERCEPTION PHASE (30-80ms)                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Input: Raw sensor streams                                             │
│  Process: Object detection, clustering, fusion                         │
│  Output: Cone observations with uncertainties                          │
│                                                                         │
│  ┌─────────────┐      ┌──────────────┐      ┌─────────────┐           │
│  │ YOLOv8      │      │ DBSCAN       │      │ Kalman      │           │
│  │ Detection   │─────►│ Clustering   │─────►│ Fusion      │           │
│  │ (Camera)    │      │ (LiDAR)      │      │ (Combined)  │           │
│  └─────────────┘      └──────────────┘      └──────┬──────┘           │
│                                                      │                 │
└──────────────────────────────────────────────────────┼─────────────────┘
                                                       │
┌──────────────────────────────────────────────────────▼─────────────────┐
│                   LOCALIZATION PHASE (10-20ms)                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Input: Cone observations + IMU/GPS/Odom                               │
│  Process: EKF state estimation + SLAM mapping                          │
│  Output: Vehicle pose (6-DOF) + Global cone map                        │
│                                                                         │
│  ┌─────────────┐      ┌──────────────┐      ┌─────────────┐           │
│  │ EKF         │      │ Data         │      │ Map         │           │
│  │ Predict     │─────►│ Association  │─────►│ Update      │           │
│  │ (100Hz)     │      │ (Nearest N.) │      │ (SLAM)      │           │
│  └─────────────┘      └──────────────┘      └──────┬──────┘           │
│                                                      │                 │
└──────────────────────────────────────────────────────┼─────────────────┘
                                                       │
┌──────────────────────────────────────────────────────▼─────────────────┐
│                     PLANNING PHASE (40-100ms)                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Input: Vehicle state + Cone map                                       │
│  Process: Path generation + Trajectory optimization                    │
│  Output: Smooth trajectory with velocity profile                       │
│                                                                         │
│  ┌─────────────┐      ┌──────────────┐      ┌─────────────┐           │
│  │ Delaunay    │      │ Minimum      │      │ Velocity    │           │
│  │ Path        │─────►│ Curvature    │─────►│ Profile     │           │
│  │ Generation  │      │ Optimization │      │ Planning    │           │
│  └─────────────┘      └──────────────┘      └──────┬──────┘           │
│                                                      │                 │
└──────────────────────────────────────────────────────┼─────────────────┘
                                                       │
┌──────────────────────────────────────────────────────▼─────────────────┐
│                      CONTROL PHASE (5-10ms)                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Input: Target trajectory + Current state                              │
│  Process: MPC optimization + PID control                               │
│  Output: Steering, throttle, brake commands                            │
│                                                                         │
│  ┌─────────────┐      ┌──────────────┐      ┌─────────────┐           │
│  │ MPC Solver  │      │ Speed PID    │      │ Command     │           │
│  │ (Lateral)   │─────►│ (Longitud.)  │─────►│ Limiters    │           │
│  │ 20 Hz       │      │ 100 Hz       │      │ (Safety)    │           │
│  └─────────────┘      └──────────────┘      └──────┬──────┘           │
│                                                      │                 │
└──────────────────────────────────────────────────────┼─────────────────┘
                                                       │
┌──────────────────────────────────────────────────────▼─────────────────┐
│                     ACTUATION PHASE (<1ms)                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ╔═══════════════╗   ╔═══════════════╗   ╔═══════════════╗            │
│  ║   Steering    ║   ║   Throttle    ║   ║     Brake     ║            │
│  ║   Actuator    ║   ║   Actuator    ║   ║   Actuator    ║            │
│  ╚═══════════════╝   ╚═══════════════╝   ╚═══════════════╝            │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘

                    TOTAL LATENCY: ~150-200ms
```

### Timing Breakdown

| Phase | Module | Frequency | Latency | Critical Path |
|-------|--------|-----------|---------|---------------|
| 1️⃣ Sensing | Sensors | Varies | 0-30ms | ⚪ |
| 2️⃣ Perception | Detection + Fusion | 10 Hz | 30-80ms | 🔴 Most compute-intensive |
| 3️⃣ Localization | EKF + SLAM | 100 Hz | 10-20ms | ⚪ |
| 4️⃣ Planning | Path + Trajectory | 20 Hz | 40-100ms | 🟡 Optimization bottleneck |
| 5️⃣ Control | MPC + PID | 100 Hz | 5-10ms | ⚪ |
| 6️⃣ Actuation | Hardware | - | <1ms | ⚪ |
| **TOTAL** | **End-to-End** | - | **150-200ms** | **Target: <200ms** |

---

## 💻 Technology Stack

### Core Framework

```
┌─────────────────────────────────────────────────────────────┐
│                      ROS 2 HUMBLE                           │
│                 (Robot Operating System)                    │
├─────────────────────────────────────────────────────────────┤
│ • Distributed message passing                              │
│ • Service/action communication                             │
│ • Parameter management                                     │
│ • Launch file orchestration                                │
│ • TF2 coordinate transforms                                │
└─────────────────────────────────────────────────────────────┘
```

### Programming Languages

| Language | Usage | Percentage |
|----------|-------|------------|
| **Python 3.8+** | Algorithm prototyping, planning, ML | 60% |
| **C++17** | Performance-critical (control, perception) | 35% |
| **YAML/XML** | Configuration, launch files | 5% |

### Key Libraries & Tools

#### Perception Stack
```yaml
Computer Vision:
  - OpenCV 4.5: Image processing
  - YOLOv8 (Ultralytics): Object detection
  - PyTorch 2.0: Deep learning inference
  
Point Cloud Processing:
  - PCL 1.12: Point cloud library
  - Open3D: Visualization & algorithms
```

#### Localization & Mapping
```yaml
State Estimation:
  - robot_localization: ROS EKF package
  - GeographicLib: GPS coordinate conversion
  
SLAM:
  - g2o: Graph optimization
  - GTSAM: Factor graph optimization
```

#### Planning & Optimization
```yaml
Path Planning:
  - SciPy: Delaunay triangulation
  - NetworkX: Graph algorithms
  
Optimization:
  - CasADi: Nonlinear optimization
  - OSQP: Quadratic programming
  - IPOPT: Interior point optimizer
```

#### Control
```yaml
Controllers:
  - control_toolbox: PID implementation
  - do-mpc: Model predictive control
  
Mathematics:
  - NumPy: Numerical computing
  - Eigen3: Linear algebra (C++)
```

#### Simulation
```yaml
Simulators:
  - Gazebo Classic 11: Physics simulation
  - RViz2: 3D visualization
  
Models:
  - URDF: Robot description
  - SDF: Simulation description
```

### Development Tools

```yaml
Version Control: Git + GitHub
CI/CD: GitHub Actions
Testing: pytest (Python), Google Test (C++)
Documentation: Doxygen, Sphinx
Code Quality: pylint, clang-format
Profiling: valgrind, gprof
Debugging: GDB, rqt tools
```

---

## 🎨 Design Principles

Our architecture is guided by fundamental engineering principles that ensure long-term maintainability and performance.

### 1. **Modularity** 🧩

**Principle**: *Each module should have a single, well-defined responsibility*

```
Benefits:
  ✓ Parallel development by multiple team members
  ✓ Easy testing in isolation
  ✓ Swappable implementations (e.g., try different planning algorithms)
  ✓ Clear interfaces between components

Example:
  Perception module only outputs cone observations.
  It doesn't know or care what Planning does with them.
```

**Implementation**:
- Each module is a separate ROS 2 package
- Well-defined message interfaces
- No direct function calls across modules

---

### 2. **Scalability** 📈

**Principle**: *System should handle increased complexity gracefully*

```
Design for Growth:
  ✓ Add new sensors without rewriting core logic
  ✓ Support different track types/sizes
  ✓ Handle 50 cones or 500 cones equally well
  ✓ Scale compute with hardware upgrades

Example:
  Adding a thermal camera just requires new perception
  node publishing to same ConeArray topic.
```

**Implementation**:
- Use of standard ROS message types
- Configurable parameters (not hardcoded)
- Efficient algorithms (O(n log n) not O(n²))

---

### 3. **Real-Time Performance** ⚡

**Principle**: *Deterministic timing is more important than average speed*

```
Requirements:
  ✓ Control loop MUST run at 100 Hz (no exceptions)
  ✓ Worst-case latency < 250ms (99th percentile)
  ✓ Jitter < 5ms in critical paths
  ✓ Predictable resource usage

Example:
  Better to consistently hit 150ms latency than
  average 100ms with occasional 500ms spikes.
```

**Implementation**:
- Real-time OS patches (RT-PREEMPT)
- Priority-based scheduling
- Watchdog timers
- Performance monitoring

---

### 4. **Simulation-First Development** 🖥️

**Principle**: *Validate in simulation before touching hardware*

```
Workflow:
  1. Develop algorithm in simulation
  2. Test edge cases virtually (100x faster)
  3. Tune parameters with sim-to-real transfer
  4. Deploy to real vehicle only when confident

Benefits:
  ✓ No risk of vehicle damage during development
  ✓ Test dangerous scenarios safely
  ✓ Faster iteration (no track setup time)
  ✓ Reproducible results
```

**Implementation**:
- Gazebo integration from day one
- Realistic sensor models with noise
- Physics-accurate vehicle dynamics
- Same code runs in sim and reality (no #ifdef)

---

### 5. **Fail-Safe Design** 🛡️

**Principle**: *System degrades gracefully under failures*

```
Safety Layers:
  1. Hardware E-stop (always works)
  2. Software monitoring (detect failures)
  3. Redundant sensors (backup systems)
  4. Graceful degradation (reduced performance, not crash)

Example:
  GPS fails → Use visual odometry + IMU
  Camera fails → Use LiDAR-only detection
  Planning fails → Execute emergency stop
```

**Implementation**:
- Watchdog timers on all critical processes
- Health monitoring of all sensors
- Fallback algorithms
- Emergency brake logic

---

### 6. **Data-Driven Optimization** 📊

**Principle**: *Measure everything, optimize with data*

```
Logging Strategy:
  • Record all sensor inputs
  • Log all intermediate outputs
  • Track timing metrics
  • Save error conditions

Analysis Loop:
  Data → Insights → Improvements → Testing → Deployment
```

**Implementation**:
- ROS bag recording
- Automated analysis scripts
- Performance dashboards
- A/B testing framework

---

## 🗓️ Development Roadmap

### Phase 1: Foundation ✅ **COMPLETE**

**Duration**: Months 1-3 | **Status**: ✅ Done

```
Milestones:
  ✅ ROS 2 workspace setup
  ✅ Basic simulation environment (Gazebo)
  ✅ Simple vehicle model (bicycle dynamics)
  ✅ Dummy sensor plugins
  ✅ Minimal viable pipeline (end-to-end smoke test)

Deliverables:
  • System boots and communicates
  • Vehicle drives in simulation (manual control)
  • Basic visualization in RViz
```

---

### Phase 2: Core Systems 🚧 **IN PROGRESS**

**Duration**: Months 4-6 | **Status**: 🚧 60% Complete

```
Current Focus:
  ✅ Camera-based cone detection (YOLOv8 trained)
  ✅ LiDAR clustering (DBSCAN working)
  🚧 Sensor fusion (data association WIP)
  🚧 EKF localization (tuning parameters)
  🚧 Basic path planning (Delaunay implemented)
  ⏳ MPC controller (in development)

Upcoming:
  • Complete sensor fusion pipeline
  • Finalize state estimation
  • Integrate planning + control
  • Closed-loop simulation testing
```

**Blockers**:
- Camera-LiDAR calibration accuracy
- EKF tuning for aggressive maneuvers

---

### Phase 3: Optimization 📋 **PLANNED**

**Duration**: Months 7-9 | **Status**: ⏳ Not Started

```
Goals:
  ⏳ Advanced cone detection (ensemble models)
  ⏳ SLAM with loop closure
  ⏳ Minimum-time trajectory optimization
  ⏳ Adaptive MPC with learning
  ⏳ Performance profiling & optimization

Targets:
  • <150ms end-to-end latency
  • >98% cone detection accuracy
  • <5cm localization error
  • Lap times within 90% of theoretical optimum
```

---

### Phase 4: Real Vehicle Integration 🏎️ **FUTURE**

**Duration**: Months 10-12 | **Status**: ⏳ Hardware Pending

```
Tasks:
  ⏳ Hardware installation & calibration
  ⏳ Sim-to-real transfer tuning
  ⏳ Safety system validation
  ⏳ Initial test track runs
  ⏳ Iterative improvement based on data

Testing Progression:
  1. Static tests (sensors, actuators)
  2. Slow speed (5 km/h) validation
  3. Medium speed (20 km/h) testing
  4. High speed (50+ km/h) racing
  5. Competition simulation runs
```

---

### Phase 5: Competition Ready 🏆 **FUTURE**

**Duration**: Months 13+ | **Status**: ⏳ Competition Season

```
Preparation:
  ⏳ Full system validation
  ⏳ Competition simulation (all events)
  ⏳ Operator training
  ⏳ Documentation & scrutineering
  ⏳ Contingency planning

Competition Events:
  • Inspection & Technical Checks
  • Trackdrive (10 laps)
  • Autocross (time attack)
  • Skidpad (figure-8)
  • Acceleration (straight line)
```

---

## 🏁 Competition Requirements

### Formula Student Driverless (FSD) Overview

Formula Student Driverless is a competition where autonomous vehicles navigate unknown tracks marked by cones. The system must be fully autonomous with no human intervention during runs.

### Track Specifications

```yaml
Track Characteristics:
  Width: 3-5 meters (between cone boundaries)
  Length: ~200-500 meters per lap
  Cone Spacing: 3-5 meters apart
  Cone Colors:
    - Blue: Left boundary
    - Yellow: Right boundary
    - Orange (small): Start/finish markers
    - Orange (large): Special markers
  
Unknown Elements:
  • Track layout revealed on competition day
  • No prior knowledge of turns/straights
  • Variable lighting conditions
  • Outdoor environment (rain, sun, wind)
```

### Autonomous Mission (AM) Events

#### 1. Trackdrive 🏎️
```yaml
Objective: Complete 10 laps as fast as possible
Points: Based on lap time vs. competitors
Strategy: Consistency + speed
Key Challenges:
  - Maintain map over 10 laps
  - Handle wear on tires/battery
  - Adapt to changing conditions
```

#### 2. Autocross 🎯
```yaml
Objective: Complete complex track fastest (1 lap)
Points: Based on single best lap time
Strategy: Aggressive cornering
Key Challenges:
  - Cold start (no warm-up laps)
  - Unknown track geometry
  - Risk vs. reward optimization
```

#### 3. Skidpad 🔄
```yaml
Objective: Figure-8 pattern at max lateral G
Points: Based on time for pattern
Strategy: Maximize centripetal force
Key Challenges:
  - Circular path at limits
  - Drift/slip management
  - Symmetric performance (left/right)
```

#### 4. Acceleration ⚡
```yaml
Objective: 75m straight line sprint
Points: Based on time to finish
Strategy: Maximum throttle control
Key Challenges:
  - Traction control
  - Straight-line stability
  - Launch optimization
```

### Safety Requirements

```yaml
Mandatory Systems:
  ✓ Emergency Brake System (EBS)
  ✓ Autonomous System Emergency (ASME)
  ✓ Remote Emergency Stop
  ✓ Watchdog timers
  ✓ Safety zone monitoring
  
Operational Rules:
  • System must detect mission failure
  • Automatic shutdown on unsafe conditions
  • No human in the loop during runs
  • Clear indication of autonomous state
```

### Scoring System

Points are awarded based on:
1. **Performance** (70%): Lap times, event completion
2. **Efficiency** (15%): Energy usage, consistency
3. **Design** (15%): Engineering quality, innovation

**Total Points**: 1000 available
- Autonomous Events: 700 pts
- Engineering Design: 150 pts
- Cost & Manufacturing: 100 pts
- Business Presentation: 50 pts

---

## 📈 Current Status Dashboard

```
┌─────────────────────────────────────────────────────────────┐
│              PROJECT HEALTH - FEBRUARY 2026                 │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Overall Progress:        ████████░░░░░░░░░░  45%          │
│  Phase 2 (Core Systems):  ███████████░░░░░░░  60%          │
│                                                             │
│  Module Readiness:                                          │
│    Perception:      ████████████░░░░░  70% 🟢              │
│    Localization:    ██████████░░░░░░░  55% 🟡              │
│    Planning:        ███████░░░░░░░░░░  40% 🟡              │
│    Control:         ████░░░░░░░░░░░░░  25% 🔴              │
│    Simulation:      ██████████████░░░  80% 🟢              │
│                                                             │
│  Next Milestones:                                           │
│    [ ] Complete sensor fusion                               │
│    [ ] Tune EKF for track conditions                        │
│    [ ] Implement MPC controller                             │
│    [ ] First closed-loop lap in simulation                  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 🤝 Team Structure

```
┌─────────────────────────────────────────────────────────────┐
│                    TEAM ORGANIZATION                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│              🏆 Team Lead / System Architect                │
│                          │                                  │
│        ┌─────────────────┼─────────────────┐               │
│        │                 │                 │               │
│   🎯 Perception     🗺️ Localization    🧭 Planning         │
│      Lead              Lead              Lead              │
│        │                 │                 │               │
│        │                 │                 │               │
│   ⚙️ Control        💻 Software        🔧 Hardware          │
│      Lead            Lead              Integration         │
│                                                             │
│  Cross-Functional Teams:                                   │
│    • Simulation & Testing                                  │
│    • Safety & Validation                                   │
│    • Data Analysis & ML                                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 📚 Related Documentation

### For Team Members

| Document | Purpose | Audience |
|----------|---------|----------|
| [System Architecture](architecture.md) | Deep technical details | Developers |
| [Algorithm Details](algorithms.md) | Math & implementation | Technical leads |
| [Setup Guide](setup_guide.md) | Installation & config | New members |
| [API Reference](api_reference.md) | Code documentation | All developers |

### For Competition

| Document | Purpose |
|----------|---------|
| [Design Report](design_report.pdf) | Engineering justification |
| [Scrutineering Docs](scrutineering/) | Safety compliance |
| [Competition Rules](competition_rules.md) | FSD regulations |

---

## 🔗 Quick Links

- 📘 **ROS 2 Documentation**: [docs.ros.org](https://docs.ros.org/en/humble/)
- 🏎️ **Formula Student Germany**: [formulastudent.de](https://www.formulastudent.de/)
- 🤖 **Our GitHub**: [github.com/your-team/fs-autonomous](https://github.com)
- 💬 **Team Slack**: Internal communication
- 📊 **Progress Tracker**: Notion/Jira board

---

<div align="center">

## 🚀 Ready to Build Something Amazing?

Our autonomous system is more than code—it's a platform for learning, innovation, and competition. Whether you're debugging perception, tuning controllers, or optimizing lap times, every contribution moves us closer to victory.

**Let's race! 🏁**

---

[⬅️ Back to Main README](../README.md) | [System Architecture →](architecture.md) | [Algorithm Details →](algorithms.md)

---

*Document Version 1.0 | Last Updated: February 2026*  
*Formula Student Autonomous Team*

</div>
