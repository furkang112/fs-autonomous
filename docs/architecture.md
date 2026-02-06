<div align="center">

# 🏗️ System Architecture
### Formula Student Autonomous - Technical Documentation

**Version 1.0** | Last Updated: February 2026

</div>

---

## 📑 Table of Contents

1. [Overview](#-overview)
2. [System Design Philosophy](#-system-design-philosophy)
3. [Architecture Diagram](#-architecture-diagram)
4. [Module Descriptions](#-module-descriptions)
   - [Perception](#1-perception-module)
   - [Localization](#2-localization-module)
   - [Planning](#3-planning-module)
   - [Control](#4-control-module)
5. [Data Flow](#-data-flow)
6. [Communication Layer](#-communication-layer)
7. [Performance Requirements](#-performance-requirements)
8. [Safety Architecture](#-safety-architecture)

---

## 🎯 Overview

The autonomous driving system is built on a **modular, layered architecture** following the classical robotics pipeline: **Sense → Plan → Act**. Each module operates as an independent ROS 2 node, communicating through well-defined interfaces, enabling parallel development, testing, and optimization.

### Design Goals

- **Modularity**: Independent, swappable components
- **Real-time Performance**: Deterministic timing for control loops
- **Fault Tolerance**: Graceful degradation under sensor failures
- **Testability**: Each module can be tested in isolation
- **Scalability**: Easy addition of new sensors or algorithms

---

## 🧠 System Design Philosophy

```
┌─────────────────────────────────────────────────────────────┐
│                    HIERARCHICAL CONTROL                     │
├─────────────────────────────────────────────────────────────┤
│  Strategic Layer (1-5 Hz)    │  Full track planning        │
│  Tactical Layer (10-20 Hz)   │  Local trajectory           │
│  Operational Layer (50-200Hz)│  Vehicle control            │
└─────────────────────────────────────────────────────────────┘
```

### Key Principles

1. **Separation of Concerns**: Each layer handles specific abstraction level
2. **Reactive + Deliberative**: Combines fast reflexes with strategic planning
3. **Sensor Fusion First**: Multiple sensors for robustness
4. **Model-Based Control**: Physics-aware controllers for precision
5. **Fail-Safe Design**: Multiple redundancy levels

---

## 🗺️ Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           SENSOR LAYER                                   │
├─────────────┬─────────────┬──────────────┬──────────────┬───────────────┤
│   Camera    │   LiDAR     │     IMU      │   GPS/GNSS   │ Wheel Encoders│
│  (60 fps)   │  (10 Hz)    │   (100 Hz)   │   (10 Hz)    │   (100 Hz)    │
└──────┬──────┴──────┬──────┴──────┬───────┴──────┬───────┴───────┬───────┘
       │             │              │              │               │
       ▼             ▼              │              │               │
┌─────────────────────────┐         │              │               │
│   PERCEPTION MODULE     │         │              │               │
├─────────────────────────┤         │              │               │
│ • Cone Detection        │◄────────┤              │               │
│ • Color Classification  │         │              │               │
│ • 3D Position Estimation│         │              │               │
│ • Track Boundary Extract│         │              │               │
└──────────┬──────────────┘         │              │               │
           │                        │              │               │
           │  Cone Observations     │              │               │
           │  (x, y, color, cov)    │              │               │
           │                        ▼              ▼               ▼
           │              ┌────────────────────────────────────────┐
           └─────────────►│    LOCALIZATION & MAPPING MODULE      │
                          ├────────────────────────────────────────┤
                          │ • Extended Kalman Filter (EKF)         │
                          │ • Sensor Fusion (IMU+GPS+Odom)         │
                          │ • SLAM / Cone Mapping                  │
                          │ • State Estimation (x, y, θ, v)        │
                          └──────────┬─────────────────────────────┘
                                     │
                                     │  Vehicle State + Map
                                     │  (pose, velocity, cone map)
                                     │
                          ┌──────────▼─────────────────────────────┐
                          │      PLANNING MODULE                   │
                          ├────────────────────────────────────────┤
                          │ Path Planning:                         │
                          │  • Delaunay Triangulation              │
                          │  • Centerline Calculation              │
                          │  • Global Path Generation              │
                          │                                        │
                          │ Trajectory Optimization:               │
                          │  • Minimum Curvature Solver            │
                          │  • Velocity Profile Generation         │
                          │  • Smoothing & Continuity              │
                          └──────────┬─────────────────────────────┘
                                     │
                                     │  Target Trajectory
                                     │  (waypoints, velocities, curvature)
                                     │
                          ┌──────────▼─────────────────────────────┐
                          │       CONTROL MODULE                   │
                          ├────────────────────────────────────────┤
                          │ Lateral Control:                       │
                          │  • Model Predictive Control (MPC)      │
                          │  • Stanley / Pure Pursuit Fallback     │
                          │                                        │
                          │ Longitudinal Control:                  │
                          │  • Speed PID Controller                │
                          │  • Acceleration Limiter                │
                          └──────────┬─────────────────────────────┘
                                     │
                                     │  Control Commands
                                     │  (steering, throttle, brake)
                                     │
                          ┌──────────▼─────────────────────────────┐
                          │      VEHICLE INTERFACE                 │
                          ├────────────────────────────────────────┤
                          │ • CAN Bus Communication                │
                          │ • Actuator Commands                    │
                          │ • Safety Monitoring                    │
                          └────────────────────────────────────────┘
                                     │
                                     ▼
                          ┌────────────────────┐
                          │   VEHICLE ACTUATION│
                          │  Steering + Brakes │
                          └────────────────────┘

┌──────────────────────────────────────────────────────────────────────────┐
│                        CROSS-CUTTING CONCERNS                            │
├──────────────┬───────────────┬──────────────┬──────────────┬────────────┤
│ Safety       │ Diagnostics   │ Data Logging │ Visualization│ Emergency  │
│ Monitor      │ & Health      │ & Replay     │ (RViz)       │ Brake      │
└──────────────┴───────────────┴──────────────┴──────────────┴────────────┘
```

---

## 🔧 Module Descriptions

## 1️⃣ Perception Module

**Purpose**: Extract environmental understanding from raw sensor data

### 1.1 Camera-Based Cone Detection

#### Algorithm: YOLOv8 Object Detection

**Input Specifications**:
```yaml
Topic: /camera/image_raw
Type: sensor_msgs/Image
Resolution: 1920x1080 @ 60fps
Format: BGR8
Field of View: 90° horizontal
```

**Processing Pipeline**:
```
Raw Image → Preprocessing → YOLOv8 Inference → Post-processing → 2D Detections
            (resize,         (GPU)              (NMS, filtering)   (bbox, conf, class)
             normalize)
```

**Output Specifications**:
```yaml
Topic: /perception/camera/cones_2d
Type: vision_msgs/Detection2DArray
Fields:
  - bbox: [x, y, width, height]
  - confidence: float (0.0-1.0)
  - class_id: int (0=blue, 1=yellow, 2=orange, 3=large_orange)
Rate: 60 Hz
```

**Performance Requirements**:
- Detection Rate: >95% @ IoU=0.5
- False Positives: <5% per frame
- Inference Time: <16ms (GPU)
- Detection Range: 2-25 meters

---

### 1.2 LiDAR-Based Cone Detection

#### Algorithm: Point Cloud Clustering (DBSCAN/Euclidean)

**Input Specifications**:
```yaml
Topic: /lidar/points
Type: sensor_msgs/PointCloud2
Points: ~100,000 per scan
Rate: 10 Hz
Field of View: 360°
Range: 0.1 - 100m
```

**Processing Pipeline**:
```
Raw Point Cloud → Ground Removal → ROI Filter → Clustering → Cone Validation → 3D Positions
                  (RANSAC)         (distance,   (DBSCAN)    (size, shape)    (x, y, z)
                                    height)
```

**Output Specifications**:
```yaml
Topic: /perception/lidar/cones_3d
Type: geometry_msgs/PoseArray
Fields:
  - position: [x, y, z] (in vehicle frame)
  - covariance: 3x3 matrix
Rate: 10 Hz
```

**Parameters**:
```python
DBSCAN_EPSILON = 0.3        # meters
MIN_CLUSTER_SIZE = 10       # points
MAX_CLUSTER_SIZE = 200      # points
CONE_HEIGHT_RANGE = [0.25, 0.35]  # meters
CONE_RADIUS_RANGE = [0.10, 0.15]  # meters
```

---

### 1.3 Sensor Fusion & 3D Cone Localization

**Algorithm**: Camera-LiDAR Association + Triangulation

```
Camera 2D Detection + LiDAR 3D Cluster → Association → Fused Cone Observation
(pixel coords, class)  (xyz position)     (geometric)   (xyz, color, confidence)
```

**Association Method**:
1. Project 3D LiDAR points to camera frame using calibration
2. Match detections within pixel threshold (<20px)
3. Assign camera classification to LiDAR position
4. Compute observation covariance

**Output Specifications**:
```yaml
Topic: /perception/cones_fused
Type: fs_msgs/ConeArray
Fields:
  - position: [x, y] (vehicle frame, meters)
  - color: enum (BLUE, YELLOW, ORANGE_SMALL, ORANGE_LARGE, UNKNOWN)
  - confidence: float (0.0-1.0)
  - covariance: 2x2 matrix
Rate: 10 Hz
```

---

## 2️⃣ Localization Module

**Purpose**: Estimate vehicle's 6-DOF pose and velocity in global frame

### 2.1 Extended Kalman Filter (EKF)

#### State Vector (10D):
```
x = [x, y, z, roll, pitch, yaw, vx, vy, vz, yaw_rate]ᵀ
```

#### Sensor Fusion Sources:

**1. IMU (100 Hz)**:
```yaml
Measurements: [ax, ay, az, ωx, ωy, ωz]
Noise Model: Gaussian (σ_acc=0.1 m/s², σ_gyro=0.01 rad/s)
Role: High-frequency state prediction
```

**2. GPS/GNSS (10 Hz)**:
```yaml
Measurements: [latitude, longitude, altitude]
Accuracy: RTK mode: 2cm, Standard: 2-5m
Noise Model: σ_gps = 0.05m (RTK), 2.0m (standard)
Role: Absolute position correction
```

**3. Wheel Odometry (100 Hz)**:
```yaml
Measurements: [wheel_speed_FL, FR, RL, RR]
Calculation: v = (RL + RR) / 2 * wheel_radius
Noise Model: σ_odom = 0.02 m/s
Role: Velocity estimation
```

**4. Visual Odometry (Optional, 30 Hz)**:
```yaml
Method: ORB-SLAM3 / Stereo VO
Output: Relative pose transformation
Role: Backup localization in GPS-denied areas
```

#### EKF Algorithm:

**Prediction Step (100 Hz)**:
```python
def predict(dt):
    # State transition (kinematic bicycle model)
    x_pred = f(x_prev, u, dt)
    
    # Covariance prediction
    P_pred = F @ P @ F.T + Q
    
    return x_pred, P_pred
```

**Update Step (sensor-dependent rate)**:
```python
def update(z, H, R):
    # Innovation
    y = z - H @ x_pred
    S = H @ P_pred @ H.T + R
    
    # Kalman gain
    K = P_pred @ H.T @ inv(S)
    
    # State update
    x = x_pred + K @ y
    P = (I - K @ H) @ P_pred
    
    return x, P
```

**Output Specifications**:
```yaml
Topic: /localization/odometry
Type: nav_msgs/Odometry
Fields:
  - pose: [x, y, z, roll, pitch, yaw]
  - twist: [vx, vy, vz, ωx, ωy, ωz]
  - covariance: 6x6 matrices for pose and twist
Rate: 100 Hz
Accuracy: <10cm position, <2° heading
```

---

### 2.2 SLAM & Cone Mapping

**Algorithm**: EKF-SLAM with Cone Landmarks

#### Map Representation:
```python
class ConeMap:
    cones: List[Cone]  # [(x, y, color, covariance), ...]
    
class Cone:
    position: (x, y)      # Global frame (meters)
    color: enum           # BLUE | YELLOW | ORANGE
    covariance: 2x2       # Position uncertainty
    observations: int     # Number of times seen
    confidence: float     # Belief in existence (0-1)
```

#### Data Association:
```
Observed Cone → Find Nearest Map Cones → Mahalanobis Distance → Associate/Create
                (within 5m radius)        (threshold < 3σ)       (update/add)
```

**Output Specifications**:
```yaml
Topic: /localization/cone_map
Type: fs_msgs/ConeMap
Update Rate: 10 Hz
Max Map Size: 500 cones
Pruning: Remove cones not seen in 10 seconds
```

---

## 3️⃣ Planning Module

**Purpose**: Generate optimal, feasible trajectory from start to goal

### 3.1 Path Planning

#### Algorithm: Delaunay Triangulation + Centerline Extraction

**Input**:
```yaml
Topic: /localization/cone_map
Type: Cone positions grouped by color (blue_cones, yellow_cones)
```

**Processing Pipeline**:

**Step 1: Track Boundary Creation**
```python
# Separate cones by color
blue_cones = [c for c in map if c.color == BLUE]
yellow_cones = [c for c in map if c.color == YELLOW]

# Create left and right boundaries
left_boundary = fit_spline(blue_cones)
right_boundary = fit_spline(yellow_cones)
```

**Step 2: Delaunay Triangulation**
```python
# Combine all cones
all_cones = blue_cones + yellow_cones

# Compute triangulation
triangulation = scipy.spatial.Delaunay(all_cones)

# Extract valid triangles (blue-yellow edges)
valid_triangles = [
    tri for tri in triangulation 
    if has_blue_and_yellow_vertices(tri)
]
```

**Step 3: Centerline Calculation**
```python
# Compute midpoints of blue-yellow edges
centerline_points = []
for triangle in valid_triangles:
    for edge in triangle.edges:
        if connects_blue_and_yellow(edge):
            midpoint = (edge.p1 + edge.p2) / 2
            centerline_points.append(midpoint)

# Order points and smooth
centerline = order_and_smooth(centerline_points)
```

**Output**:
```yaml
Topic: /planning/centerline
Type: nav_msgs/Path
Fields: Array of [x, y] waypoints
Spacing: ~0.5 meters between points
Smoothness: C² continuous (using cubic splines)
```

---

### 3.2 Trajectory Optimization

#### Algorithm: Minimum Curvature with Velocity Profiling

**Objective**: Minimize lap time subject to vehicle constraints

**Optimization Problem**:
```
minimize:    ∫ (1/v(s)) ds          # Minimize time
             
subject to:  |κ(s)| ≤ κ_max         # Curvature limit
             |v(s)| ≤ v_max         # Speed limit
             |ay(s)| ≤ ay_max       # Lateral acceleration
             |ax(s)| ≤ ax_max       # Longitudinal acceleration
             track boundaries       # Stay within track
```

**Parameters**:
```python
V_MAX = 80 km/h           # Maximum speed
AY_MAX = 1.5 * 9.81 m/s²  # Max lateral acceleration (1.5g)
AX_MAX = 1.0 * 9.81 m/s²  # Max longitudinal acceleration
KAPPA_MAX = 1/3 m⁻¹       # Minimum turn radius: 3m
```

**Velocity Profile Calculation**:
```python
def calculate_velocity_profile(path, curvature):
    v = []
    for i, κ in enumerate(curvature):
        # Lateral acceleration constraint
        v_lat = sqrt(AY_MAX / abs(κ)) if κ != 0 else V_MAX
        
        # Apply limits
        v.append(min(v_lat, V_MAX))
    
    # Forward pass: limit acceleration
    for i in range(1, len(v)):
        ds = path[i] - path[i-1]
        v_max_accel = sqrt(v[i-1]² + 2*AX_MAX*ds)
        v[i] = min(v[i], v_max_accel)
    
    # Backward pass: limit deceleration
    for i in range(len(v)-2, -1, -1):
        ds = path[i+1] - path[i]
        v_max_decel = sqrt(v[i+1]² + 2*AX_MAX*ds)
        v[i] = min(v[i], v_max_decel)
    
    return v
```

**Output**:
```yaml
Topic: /planning/trajectory
Type: fs_msgs/Trajectory
Fields:
  - waypoints: Array of [x, y]
  - velocities: Array of target speeds (m/s)
  - curvatures: Array of path curvature (1/m)
  - timestamps: Estimated arrival time at each point
Rate: 20 Hz
Lookahead: 50 meters or 100 waypoints
```

---

## 4️⃣ Control Module

**Purpose**: Track planned trajectory with high precision

### 4.1 Lateral Control - Model Predictive Control (MPC)

**Vehicle Model**: Kinematic Bicycle Model
```
ẋ = v·cos(ψ)
ẏ = v·sin(ψ)
ψ̇ = (v/L)·tan(δ)

where:
  (x,y): position
  ψ: heading angle
  v: velocity
  δ: steering angle
  L: wheelbase (1.6m)
```

**MPC Formulation**:
```
minimize:  Σ(||e_lat||²·Q + ||e_heading||²·Q_ψ + ||δ||²·R + ||Δδ||²·R_Δ)

subject to:
  x_{k+1} = f(x_k, u_k)        # Vehicle dynamics
  |δ| ≤ δ_max = 25°            # Steering limit
  |Δδ| ≤ Δδ_max = 10°/sample   # Steering rate limit
```

**Parameters**:
```python
PREDICTION_HORIZON = 20       # 20 time steps
CONTROL_HORIZON = 10          # 10 control inputs
DT = 0.05                     # 50ms sample time
FREQUENCY = 20 Hz

# Cost weights
Q_LAT = 100                   # Lateral error weight
Q_HEADING = 10                # Heading error weight
R_STEERING = 1                # Steering effort weight
R_DELTA_STEERING = 10         # Steering rate weight
```

**Implementation**:
```python
def mpc_control(state, trajectory, horizon):
    # Extract reference from trajectory
    ref_points = get_reference_points(trajectory, horizon)
    
    # Setup optimization problem
    opti = casadi.Opti()
    X = opti.variable(4, horizon+1)  # [x, y, ψ, v]
    U = opti.variable(1, horizon)    # [δ]
    
    # Cost function
    cost = 0
    for k in range(horizon):
        e_lat = lateral_error(X[:,k], ref_points[k])
        e_ψ = heading_error(X[2,k], ref_points[k])
        cost += Q_LAT*e_lat**2 + Q_HEADING*e_ψ**2
        cost += R_STEERING*U[0,k]**2
        if k > 0:
            cost += R_DELTA_STEERING*(U[0,k]-U[0,k-1])**2
    
    opti.minimize(cost)
    
    # Dynamics constraints
    for k in range(horizon):
        x_next = kinematic_model(X[:,k], U[:,k], DT)
        opti.subject_to(X[:,k+1] == x_next)
    
    # Input constraints
    opti.subject_to(opti.bounded(-δ_max, U, δ_max))
    
    # Solve
    solution = opti.solve()
    return solution.value(U[0,0])  # Return first control input
```

**Output**:
```yaml
Topic: /control/steering_cmd
Type: std_msgs/Float64
Range: [-25°, +25°] (± 0.436 rad)
Rate: 100 Hz (control runs at 20Hz, interpolated to 100Hz)
```

---

### 4.2 Longitudinal Control - Speed PID Controller

**Control Law**:
```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·ė(t)

where:
  e(t) = v_target - v_actual
  u(t) = throttle/brake command
```

**Parameters**:
```python
KP = 0.8        # Proportional gain
KI = 0.1        # Integral gain
KD = 0.2        # Derivative gain

# Anti-windup
MAX_INTEGRAL = 5.0
MIN_INTEGRAL = -5.0

# Output limits
THROTTLE_MAX = 1.0   # 100%
BRAKE_MAX = 1.0      # 100%
```

**Safety Features**:
```python
# Emergency brake trigger
if obstacle_detected() or emergency_stop:
    return BRAKE_MAX

# Speed limiting
if v_actual > V_MAX * 1.1:  # 10% overspeed
    return BRAKE_MAX

# Acceleration limiting
if acceleration > AX_MAX:
    throttle = limit_acceleration(throttle)
```

**Output**:
```yaml
Topics:
  /control/throttle_cmd: std_msgs/Float64 [0.0, 1.0]
  /control/brake_cmd: std_msgs/Float64 [0.0, 1.0]
Rate: 100 Hz
```

---

## 📊 Data Flow

### Message Flow Diagram

```
┌────────────────┐     ┌────────────────┐     ┌────────────────┐
│ Camera (60Hz)  │────►│ Perception     │────►│ Localization   │
└────────────────┘     │                │     │                │
                       │ • Cone Det     │     │ • EKF Filter   │
┌────────────────┐     │ • Fusion       │     │ • SLAM Map     │
│ LiDAR (10Hz)   │────►│ • Tracking     │────►│ • State Est    │
└────────────────┘     └────────────────┘     └────────┬───────┘
                                                        │
┌────────────────┐                                     │
│ IMU (100Hz)    │────────────────────────────────────►│
└────────────────┘                                     │
                                                        │
┌────────────────┐                                     │
│ GPS (10Hz)     │────────────────────────────────────►│
└────────────────┘                                     │
                                                        ▼
                                              ┌─────────────────┐
                                              │ Planning        │
                                              │                 │
                                              │ • Path Gen      │
                                              │ • Trajectory    │
                                              │ • Velocity      │
                                              └────────┬────────┘
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │ Control         │
                                              │                 │
                                              │ • MPC           │
                                              │ • Speed PID     │
                                              └────────┬────────┘
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │ Actuators       │
                                              │ Steer + Brake   │
                                              └─────────────────┘
```

### Topic List & Rates

| Topic Name | Message Type | Rate | Description |
|------------|-------------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | 60 Hz | Raw camera frames |
| `/lidar/points` | sensor_msgs/PointCloud2 | 10 Hz | 3D point cloud |
| `/imu/data` | sensor_msgs/Imu | 100 Hz | IMU measurements |
| `/gps/fix` | sensor_msgs/NavSatFix | 10 Hz | GPS position |
| `/perception/cones_fused` | fs_msgs/ConeArray | 10 Hz | Detected cones |
| `/localization/odometry` | nav_msgs/Odometry | 100 Hz | Vehicle state |
| `/localization/cone_map` | fs_msgs/ConeMap | 10 Hz | Global map |
| `/planning/trajectory` | fs_msgs/Trajectory | 20 Hz | Target trajectory |
| `/control/steering_cmd` | std_msgs/Float64 | 100 Hz | Steering angle |
| `/control/throttle_cmd` | std_msgs/Float64 | 100 Hz | Throttle position |
| `/control/brake_cmd` | std_msgs/Float64 | 100 Hz | Brake pressure |

---

## 🔌 Communication Layer

### ROS 2 Infrastructure

**DDS Middleware**: FastDDS (default)

**QoS Profiles**:

```python
# Sensor data (best effort, volatile)
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# Control commands (reliable, volatile)
control_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# Map data (reliable, transient local)
map_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)
```

---

## ⚡ Performance Requirements

| Module | Frequency | Latency | CPU Usage | Memory |
|--------|-----------|---------|-----------|--------|
| Perception | 10-60 Hz | <50ms | 40% (GPU) | 2 GB |
| Localization | 100 Hz | <10ms | 15% | 500 MB |
| Planning | 20 Hz | <100ms | 20% | 1 GB |
| Control | 100 Hz | <5ms | 10% | 200 MB |
| **Total** | - | **<200ms** | **85%** | **3.7 GB** |

**Hardware Requirements**:
- CPU: Intel i7 or equivalent (4+ cores)
- GPU: NVIDIA RTX 2060 or better (for perception)
- RAM: 16 GB minimum
- Storage: 256 GB SSD (for data logging)

---

## 🛡️ Safety Architecture

### Safety Layers

```
┌─────────────────────────────────────────────────────────────┐
│ Layer 1: Emergency Stop (Hardware)                         │
│ • Physical E-Stop button                                   │
│ • Wireless remote kill switch                              │
│ • Watchdog timer (100ms timeout)                           │
├─────────────────────────────────────────────────────────────┤
│ Layer 2: Safety Monitor (Software)                         │
│ • Sensor health checks                                     │
│ • State estimation validation                              │
│ • Trajectory feasibility checks                            │
│ • Speed limiting                                           │
├─────────────────────────────────────────────────────────────┤
│ Layer 3: Operational Limits                                │
│ • Geofence boundaries                                      │
│ • Maximum speed enforcement                                │
│ • Steering angle limits                                    │
│ • Acceleration limits                                      │
└─────────────────────────────────────────────────────────────┘
```

### Safety Conditions

**Emergency Brake Triggers**:
```python
EMERGENCY_BRAKE = (
    no_cones_detected_for(2.0) or          # Lost track
    localization_uncertainty > 1.0 or       # Position uncertain
    trajectory_collision_risk() or          # Collision predicted
    speed > V_MAX * 1.2 or                 # Overspeed
    sensor_failure_detected() or           # Sensor fault
    communication_timeout() or             # System fault
    manual_estop_pressed()                 # Manual trigger
)
```

**Degraded Mode**:
- If GPS fails → Use visual odometry + IMU
- If camera fails → Use LiDAR-only detection
- If planning fails → Execute emergency stop trajectory

---

## 📐 Coordinate Frames

### Frame Definitions

```
world (global):
  - Origin: Race start line
  - X: East, Y: North, Z: Up
  - Fixed inertial frame

vehicle (base_link):
  - Origin: Rear axle center
  - X: Forward, Y: Left, Z: Up
  - Moving with vehicle

camera:
  - Origin: Camera optical center
  - X: Right, Y: Down, Z: Forward (optical)

lidar:
  - Origin: LiDAR sensor center
  - X: Forward, Y: Left, Z: Up
```

### Transform Tree
```
world
 └─ odom
     └─ base_link
         ├─ camera_link
         ├─ lidar_link
         └─ imu_link
```

---

## 🔍 Debugging & Visualization

### RViz Configuration

**Displays**:
- `/camera/image_raw` → Camera feed with detections
- `/lidar/points` → Point cloud (colored by height)
- `/perception/cones_fused` → 3D cone markers (colored)
- `/localization/cone_map` → Global map visualization
- `/planning/trajectory` → Path (green line)
- `/localization/odometry` → Vehicle pose (coordinate frame)
- `/tf` → Transform tree

**Useful Views**:
- Top-down (planning) view
- Driver perspective (3D)
- Sensor overlay (debug)

---

## 📚 References

1. **Perception**: YOLOv8 - Ultralytics Documentation
2. **Localization**: Probabilistic Robotics (Thrun, Burgard, Fox)
3. **Planning**: Optimal Trajectory Generation (Katrakazas et al.)
4. **Control**: Model Predictive Control (Camacho & Bordons)
5. **Formula Student**: FSG Rules 2024

---

<div align="center">

**For detailed algorithm implementations, see individual module documentation**

[← Back to Main README](../README.md) | [Algorithm Details →](algorithms.md)

*Last Updated: February 2026 | Version 1.0*

</div>
