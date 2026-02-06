<div align="center">

# 🗓️ Development Roadmap
### Formula Student Autonomous - Project Planning & Execution

**Version 1.0** | Last Updated: February 2026

*Your guide to building a competitive autonomous racing system*

</div>

---

## 📑 Table of Contents

1. [Overview](#-overview)
2. [Timeline](#-timeline)
3. [Phase 1: Foundation & Setup](#phase-1-foundation--setup)
4. [Phase 2: Perception System](#phase-2-perception-system)
5. [Phase 3: Localization & Mapping](#phase-3-localization--mapping)
6. [Phase 4: Planning & Trajectory](#phase-4-planning--trajectory)
7. [Phase 5: Control System](#phase-5-control-system)
8. [Phase 6: Integration & Testing](#phase-6-integration--testing)
9. [Phase 7: Competition Preparation](#phase-7-competition-preparation)
10. [Resources & Best Practices](#-resources--best-practices)

---

## 🎯 Overview

This roadmap provides a structured approach to developing our Formula Student Autonomous system from scratch to competition-ready. Each phase builds upon the previous, with clear deliverables, acceptance criteria, and dependencies.

### Success Metrics

```
┌─────────────────────────────────────────────────────────────┐
│                   PROJECT SUCCESS CRITERIA                  │
├─────────────────────────────────────────────────────────────┤
│  ✅ Complete autonomous lap in simulation                   │
│  ✅ All safety systems validated                            │
│  ✅ <200ms end-to-end latency                               │
│  ✅ >95% cone detection accuracy                            │
│  ✅ <10cm localization error                                │
│  ✅ Hardware integration complete                           │
│  ✅ Competition documentation ready                         │
└─────────────────────────────────────────────────────────────┘
```

---

## 📅 Timeline

### Master Schedule (12-Month Plan)

```
Month  │ Phase                           │ Key Milestones
───────┼─────────────────────────────────┼──────────────────────────────
  1    │ ████ Phase 1: Setup             │ ✓ Repo + Docs + Sim
  2    │ ████ Phase 1 (continued)        │ ✓ Basic infrastructure
  ─────┼─────────────────────────────────┼──────────────────────────────
  3    │ ████ Phase 2: Perception        │ Camera detection working
  4    │ ████ Phase 2 (continued)        │ ✓ LiDAR + Fusion complete
  ─────┼─────────────────────────────────┼──────────────────────────────
  5    │ ████ Phase 3: Localization      │ EKF state estimation
  6    │ ████ Phase 3 (continued)        │ ✓ SLAM mapping working
  ─────┼─────────────────────────────────┼──────────────────────────────
  7    │ ████ Phase 4: Planning          │ Path generation working
  8    │ ████ Phase 4 (continued)        │ ✓ Trajectory optimization
  ─────┼─────────────────────────────────┼──────────────────────────────
  9    │ ████ Phase 5: Control           │ MPC controller working
 10    │ ████ Phase 5 (continued)        │ ✓ Full control stack
  ─────┼─────────────────────────────────┼──────────────────────────────
 11    │ ████ Phase 6: Integration       │ Hardware + Software
 12    │ ████ Phase 7: Competition Prep  │ ✓ Ready to race!
```

### Critical Path Dependencies

```
Setup → Perception → Localization → Planning → Control → Integration → Competition
  │         │            │             │          │           │
  └─────────┴────────────┴─────────────┴──────────┴───────────┘
           Simulation Environment (runs parallel)
```

---

## Phase 1: Foundation & Setup

**Duration**: 2 months (Month 1-2)  
**Status**: ✅ Complete  
**Team Focus**: All members

### 🎯 Objectives

Build the foundation for all future development work with proper infrastructure, documentation, and a working simulation environment.

---

### 📦 Task 1.1: Repository Structure

**Owner**: Software Lead  
**Duration**: 1 week  
**Priority**: 🔴 Critical

#### Deliverables

```bash
fs-autonomous/
├── .github/
│   ├── workflows/          # CI/CD pipelines
│   └── ISSUE_TEMPLATE/     # Issue templates
├── docs/
│   ├── README.md
│   ├── architecture.md
│   ├── setup_guide.md
│   └── api_reference.md
├── perception/
├── localization/
├── planning/
├── control/
├── simulation/
├── utils/
├── tests/
├── config/
├── launch/
├── requirements.txt
├── package.xml
└── CMakeLists.txt
```

#### Action Items

- [ ] Initialize Git repository
- [ ] Create branch protection rules (main, develop)
- [ ] Set up .gitignore for ROS, Python, C++
- [ ] Create directory structure
- [ ] Add LICENSE file (MIT recommended)
- [ ] Create CONTRIBUTING.md guidelines

#### Acceptance Criteria

✅ Repository has clear structure  
✅ CI/CD pipeline runs successfully  
✅ All team members can clone and build  

---

### 📚 Task 1.2: Documentation Setup

**Owner**: Team Lead  
**Duration**: 1 week  
**Priority**: 🟡 High

#### Deliverables

1. **README.md** - Project overview, features, quick start
2. **architecture.md** - System design, modules, data flow
3. **setup_guide.md** - Installation instructions
4. **development_roadmap.md** - This document!
5. **competition_rules.md** - FSG/FSA rules summary

#### Action Items

- [ ] Write comprehensive README with badges
- [ ] Document system architecture with diagrams
- [ ] Create step-by-step setup guide
- [ ] Document competition requirements
- [ ] Set up auto-generated API docs (Doxygen)

#### Acceptance Criteria

✅ New member can set up environment using docs alone  
✅ Architecture is clearly explained with visuals  
✅ Competition rules are documented  

---

### 🖥️ Task 1.3: Simulation Environment

**Owner**: Simulation Lead  
**Duration**: 3 weeks  
**Priority**: 🔴 Critical

#### Deliverables

**1. Gazebo World Setup**

```yaml
Worlds to Create:
  - simple_track.world      # Oval for basic testing
  - fsg_track.world         # Realistic FSG layout
  - test_scenarios.world    # Edge cases (tight turns, etc.)
```

**2. Vehicle Model (URDF/SDF)**

```yaml
Components:
  - Chassis (base_link)
  - Wheels (4x, with suspension)
  - Sensors:
      * Camera (front-facing, 90° FOV)
      * LiDAR (360°, 100m range)
      * IMU (100 Hz)
      * GPS (10 Hz, with noise)
      * Wheel encoders
  - Actuators:
      * Steering (Ackermann)
      * Throttle/Brake
```

**3. Cone Models**

```yaml
Cone Types:
  - blue_cone.dae       # Left boundary (0.325m tall)
  - yellow_cone.dae     # Right boundary (0.325m tall)
  - orange_cone_small   # Start/finish
  - orange_cone_large   # Special markers
```

#### Action Items

- [ ] Install Gazebo Classic 11
- [ ] Create vehicle URDF with accurate dimensions
- [ ] Model sensors with realistic noise
- [ ] Create cone 3D models (or download)
- [ ] Build test track worlds
- [ ] Configure physics parameters (friction, inertia)
- [ ] Set up RViz visualization config
- [ ] Create launch files for different scenarios

#### Code Example: Launch File

```python
# launch/sim_basic.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Gazebo
        IncludeLaunchDescription(
            'gazebo_ros/launch/gazebo.launch.py',
            launch_arguments={'world': 'simple_track.world'}.items()
        ),
        
        # Spawn vehicle
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'fs_car', '-file', 'vehicle.urdf']
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'config/sim_view.rviz']
        ),
    ])
```

#### Acceptance Criteria

✅ Vehicle spawns correctly in Gazebo  
✅ All sensors publish data on correct topics  
✅ Manual control works (keyboard/joystick)  
✅ RViz shows camera, LiDAR, TF tree  
✅ Physics behaves realistically (no jittering)  

---

### 🔧 Task 1.4: Development Environment

**Owner**: All Team Members  
**Duration**: 1 week  
**Priority**: 🔴 Critical

#### Action Items

- [ ] Install Ubuntu 22.04 LTS (dual boot or VM)
- [ ] Install ROS 2 Humble
- [ ] Install dependencies (see setup_guide.md)
- [ ] Configure bashrc with ROS sourcing
- [ ] Install IDE (VS Code recommended)
- [ ] Install ROS extensions for IDE
- [ ] Set up Python virtual environment
- [ ] Configure code formatters (black, clang-format)

#### Setup Script

```bash
#!/bin/bash
# setup_environment.sh

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
sudo apt install -y ros-humble-desktop

# Install dependencies
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    gazebo11 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-localization \
    ros-humble-tf2-tools

# Python packages
pip3 install --user \
    numpy \
    scipy \
    opencv-python \
    matplotlib \
    ultralytics  # YOLOv8

# Initialize rosdep
sudo rosdep init
rosdep update

echo "Setup complete! Source your workspace:"
echo "source /opt/ros/humble/setup.bash"
```

#### Acceptance Criteria

✅ ROS 2 commands work (`ros2 topic list`)  
✅ Gazebo launches without errors  
✅ Can build workspace with `colcon build`  

---

### 📊 Phase 1 Milestones

| Milestone | Completion Criteria | Status |
|-----------|---------------------|--------|
| M1.1 - Infrastructure Ready | Repo structure + CI/CD working | ✅ |
| M1.2 - Docs Published | All core documentation written | ✅ |
| M1.3 - Simulation Running | Vehicle drives in Gazebo | ✅ |
| M1.4 - Team Onboarded | All members can build & run | ✅ |

---

## Phase 2: Perception System

**Duration**: 2 months (Month 3-4)  
**Status**: 🚧 In Progress (70% complete)  
**Team Focus**: Perception Lead + 2 developers

### 🎯 Objectives

Develop robust cone detection using camera and LiDAR, with sensor fusion to produce accurate 3D positions and color classifications.

---

### 📷 Task 2.1: Camera-Based Cone Detection

**Owner**: Perception Lead  
**Duration**: 3 weeks  
**Priority**: 🔴 Critical

#### Deliverables

**1. Dataset Preparation**

```yaml
Data Collection:
  Sources:
    - Simulation (Gazebo screenshots)
    - Previous competitions (if available)
    - Public FS datasets
  
  Augmentation:
    - Brightness/contrast variations
    - Different lighting conditions
    - Motion blur
    - Random occlusions
  
  Annotations:
    Tool: LabelImg or Roboflow
    Format: YOLO format (class, x, y, w, h)
    Classes: [blue_cone, yellow_cone, orange_small, orange_large]
```

**2. Model Training (YOLOv8)**

```python
# train_detector.py
from ultralytics import YOLO

# Load pretrained model
model = YOLO('yolov8n.pt')  # Nano for speed

# Train
results = model.train(
    data='cone_dataset.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    name='cone_detector_v1',
    patience=20,
    device=0  # GPU
)

# Validate
metrics = model.val()
print(f"mAP@50: {metrics.box.map50}")
print(f"mAP@50-95: {metrics.box.map}")

# Export for inference
model.export(format='onnx')  # For fast deployment
```

**3. ROS 2 Detection Node**

```python
# perception/camera/cone_detector_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ConeDetectorNode(Node):
    def __init__(self):
        super().__init__('cone_detector')
        
        # Load model
        model_path = self.declare_parameter('model_path', 'cone_detector.pt').value
        self.model = YOLO(model_path)
        
        # Publishers/Subscribers
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(
            Detection2DArray, '/perception/camera/detections', 10)
        
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run inference
        results = self.model(cv_image, conf=0.5, iou=0.4)
        
        # Convert to Detection2DArray
        detections = Detection2DArray()
        detections.header = msg.header
        
        for r in results:
            for box in r.boxes:
                det = Detection2D()
                det.bbox.center.x = box.xywh[0][0]
                det.bbox.center.y = box.xywh[0][1]
                det.bbox.size_x = box.xywh[0][2]
                det.bbox.size_y = box.xywh[0][3]
                
                # Class mapping: 0=blue, 1=yellow, 2=orange
                det.results[0].id = int(box.cls[0])
                det.results[0].score = float(box.conf[0])
                
                detections.detections.append(det)
        
        self.pub.publish(detections)
```

#### Action Items

- [ ] Collect/download 2000+ labeled cone images
- [ ] Split dataset (80% train, 10% val, 10% test)
- [ ] Train YOLOv8 model (target mAP@50 > 0.90)
- [ ] Implement ROS 2 detection node
- [ ] Optimize for real-time (>30 FPS)
- [ ] Add confidence filtering (reject < 0.5)
- [ ] Test in different lighting conditions

#### Acceptance Criteria

✅ Detection rate >95% on test set  
✅ False positives <5%  
✅ Real-time performance (>30 FPS on GPU)  
✅ Works in simulation environment  

---

### 🔷 Task 2.2: LiDAR-Based Cone Detection

**Owner**: Perception Team  
**Duration**: 2 weeks  
**Priority**: 🟡 High

#### Deliverables

**1. Point Cloud Processing**

```python
# perception/lidar/lidar_detector_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from sklearn.cluster import DBSCAN

class LidarDetectorNode(Node):
    def __init__(self):
        super().__init__('lidar_detector')
        
        self.sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.pointcloud_callback, 10)
        self.pub = self.create_publisher(
            PoseArray, '/perception/lidar/cones_3d', 10)
        
        # Parameters
        self.eps = 0.3  # DBSCAN epsilon (meters)
        self.min_samples = 10
        self.cone_height_range = (0.25, 0.35)  # meters
        
    def pointcloud_callback(self, msg):
        # Convert to numpy array
        points = np.array(list(pc2.read_points(msg, field_names=['x', 'y', 'z'])))
        
        # 1. Ground removal (RANSAC plane fitting)
        non_ground = self.remove_ground(points)
        
        # 2. ROI filtering (distance + height)
        roi_points = non_ground[
            (non_ground[:, 2] > 0.1) &  # Above ground
            (non_ground[:, 2] < 0.5) &  # Below max cone height
            (np.linalg.norm(non_ground[:, :2], axis=1) < 20)  # Within 20m
        ]
        
        # 3. Clustering (DBSCAN)
        if len(roi_points) < self.min_samples:
            return
        
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples)
        labels = clustering.fit_predict(roi_points[:, :2])  # Only x, y
        
        # 4. Validate clusters (cone-like geometry)
        cone_poses = PoseArray()
        cone_poses.header = msg.header
        
        for cluster_id in set(labels):
            if cluster_id == -1:  # Noise
                continue
            
            cluster = roi_points[labels == cluster_id]
            
            # Check if cone-like
            if self.is_cone_like(cluster):
                pose = Pose()
                pose.position.x = float(np.mean(cluster[:, 0]))
                pose.position.y = float(np.mean(cluster[:, 1]))
                pose.position.z = 0.0
                cone_poses.poses.append(pose)
        
        self.pub.publish(cone_poses)
    
    def remove_ground(self, points):
        # Simple height threshold (can upgrade to RANSAC)
        return points[points[:, 2] > 0.05]
    
    def is_cone_like(self, cluster):
        # Check cluster size
        if len(cluster) < 10 or len(cluster) > 200:
            return False
        
        # Check height
        height = cluster[:, 2].max() - cluster[:, 2].min()
        if not (self.cone_height_range[0] < height < self.cone_height_range[1]):
            return False
        
        # Check diameter (should be ~0.2m)
        diameter = np.ptp(cluster[:, :2], axis=0).max()
        if diameter > 0.3:
            return False
        
        return True
```

#### Action Items

- [ ] Implement ground removal (RANSAC or height-based)
- [ ] Implement DBSCAN clustering
- [ ] Add geometric validation (height, diameter)
- [ ] Tune parameters (epsilon, min_samples)
- [ ] Test with various cone distances
- [ ] Handle edge cases (overlapping cones)

#### Acceptance Criteria

✅ Detects cones 2-20m away  
✅ Clustering works reliably  
✅ False positive rate <10%  
✅ Processing time <50ms per scan  

---

### 🔀 Task 2.3: Sensor Fusion (Camera + LiDAR)

**Owner**: Perception Lead  
**Duration**: 2 weeks  
**Priority**: 🔴 Critical

#### Deliverables

**1. Camera-LiDAR Calibration**

```yaml
Calibration Process:
  1. Place checkerboard in view of both sensors
  2. Collect synchronized data (camera + LiDAR)
  3. Use kalibr or similar tool
  4. Output: Transformation matrix T_lidar_camera
  
Result: 4x4 transformation matrix
```

**2. Data Association & Fusion**

```python
# perception/fusion/sensor_fusion_node.py
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray
from fs_msgs.msg import ConeArray, Cone
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Detection2DArray, '/perception/camera/detections',
            self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            PoseArray, '/perception/lidar/cones_3d',
            self.lidar_callback, 10)
        
        # Publisher
        self.pub = self.create_publisher(
            ConeArray, '/perception/cones_fused', 10)
        
        # Buffers
        self.camera_detections = None
        self.lidar_cones = None
        
        # Calibration (from calibration file)
        self.T_lidar_camera = self.load_calibration()
        self.camera_matrix = self.load_camera_intrinsics()
        
        # Timer for fusion (10 Hz)
        self.create_timer(0.1, self.fusion_callback)
    
    def camera_callback(self, msg):
        self.camera_detections = msg
    
    def lidar_callback(self, msg):
        self.lidar_cones = msg
    
    def fusion_callback(self):
        if self.camera_detections is None or self.lidar_cones is None:
            return
        
        fused_cones = ConeArray()
        
        for lidar_cone in self.lidar_cones.poses:
            # Project 3D LiDAR point to camera frame
            pixel = self.project_to_image(lidar_cone.position)
            
            # Find matching camera detection
            match = self.find_best_match(pixel, self.camera_detections)
            
            if match is not None:
                cone = Cone()
                cone.position.x = lidar_cone.position.x
                cone.position.y = lidar_cone.position.y
                cone.color = self.get_color_from_class(match.results[0].id)
                cone.confidence = match.results[0].score
                
                # TODO: Compute covariance
                cone.covariance = self.compute_covariance(cone)
                
                fused_cones.cones.append(cone)
        
        self.pub.publish(fused_cones)
    
    def project_to_image(self, point_3d):
        """Project 3D point to 2D pixel coordinates"""
        # Transform to camera frame
        point_cam = self.T_lidar_camera @ np.array([
            point_3d.x, point_3d.y, point_3d.z, 1.0
        ])
        
        # Project using camera matrix
        pixel_homog = self.camera_matrix @ point_cam[:3]
        pixel = pixel_homog[:2] / pixel_homog[2]
        
        return pixel
    
    def find_best_match(self, pixel, detections, threshold=20):
        """Find camera detection closest to projected pixel"""
        best_match = None
        best_dist = threshold
        
        for det in detections.detections:
            det_pixel = np.array([det.bbox.center.x, det.bbox.center.y])
            dist = np.linalg.norm(pixel - det_pixel)
            
            if dist < best_dist:
                best_dist = dist
                best_match = det
        
        return best_match
    
    def get_color_from_class(self, class_id):
        """Map class ID to cone color"""
        color_map = {
            0: Cone.BLUE,
            1: Cone.YELLOW,
            2: Cone.ORANGE_SMALL,
            3: Cone.ORANGE_LARGE
        }
        return color_map.get(class_id, Cone.UNKNOWN)
```

#### Action Items

- [ ] Calibrate camera-LiDAR extrinsics
- [ ] Implement 3D→2D projection
- [ ] Implement data association algorithm
- [ ] Handle temporal synchronization (message_filters)
- [ ] Add covariance estimation
- [ ] Test fusion accuracy

#### Acceptance Criteria

✅ >90% correct associations  
✅ Position accuracy <0.2m  
✅ Color classification >95% accurate  
✅ Runs at 10 Hz reliably  

---

### 📊 Phase 2 Milestones

| Milestone | Completion Criteria | Target | Status |
|-----------|---------------------|--------|--------|
| M2.1 - Camera Detection | YOLOv8 trained, mAP>0.90 | Week 3 | ✅ |
| M2.2 - LiDAR Detection | Clustering working reliably | Week 5 | ✅ |
| M2.3 - Sensor Fusion | Fused output <0.2m error | Week 7 | 🚧 |
| M2.4 - Perception Complete | All tests passing | Week 8 | ⏳ |

---

## Phase 3: Localization & Mapping

**Duration**: 2 months (Month 5-6)  
**Status**: 🚧 In Progress (55% complete)  
**Team Focus**: Localization Lead + 2 developers

### 🎯 Objectives

Implement robust state estimation using sensor fusion (EKF) and build a global map of cones (SLAM).

---

### 🧭 Task 3.1: Extended Kalman Filter (EKF)

**Owner**: Localization Lead  
**Duration**: 3 weeks  
**Priority**: 🔴 Critical

#### Deliverables

**1. State Vector Design**

```python
# State: [x, y, z, roll, pitch, yaw, vx, vy, vz, ωz]
STATE_DIM = 10

# Measurement vectors
IMU_MEAS = 6    # [ax, ay, az, ωx, ωy, ωz]
GPS_MEAS = 3    # [lat, lon, alt]
ODOM_MEAS = 1   # [v]
```

**2. EKF Implementation**

```python
# localization/ekf/ekf_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np

class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization')
        
        # State
        self.x = np.zeros(10)  # [x, y, z, roll, pitch, yaw, vx, vy, vz, ωz]
        self.P = np.eye(10) * 1.0  # Covariance
        
        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.5, 0.5, 0.5, 0.1])
        
        # Measurement noise
        self.R_imu = np.diag([0.1]*6)
        self.R_gps = np.diag([2.0, 2.0, 5.0])  # meters
        self.R_odom = np.array([[0.02]])
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(
            TwistWithCovarianceStamped, '/wheel_odom',
            self.odom_callback, 10)
        
        # Publisher
        self.pub = self.create_publisher(
            Odometry, '/localization/odometry', 10)
        
        # Timer (100 Hz prediction)
        self.dt = 0.01
        self.create_timer(self.dt, self.predict_step)
        
        self.last_time = self.get_clock().now()
    
    def predict_step(self):
        """EKF Prediction using kinematic model"""
        # Extract state
        x, y, z = self.x[0:3]
        roll, pitch, yaw = self.x[3:6]
        vx, vy, vz = self.x[6:9]
        omega_z = self.x[9]
        
        # State transition (simple kinematic model)
        self.x[0] += vx * np.cos(yaw) * self.dt - vy * np.sin(yaw) * self.dt
        self.x[1] += vx * np.sin(yaw) * self.dt + vy * np.cos(yaw) * self.dt
        self.x[2] += vz * self.dt
        self.x[5] += omega_z * self.dt  # yaw
        
        # Jacobian F (linearization of state transition)
        F = self.compute_state_jacobian()
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q
        
        # Publish
        self.publish_state()
    
    def imu_callback(self, msg):
        """Update step with IMU measurement"""
        # Measurement: [ax, ay, az, ωx, ωy, ωz]
        z = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Measurement model H (how measurement relates to state)
        H = self.compute_imu_jacobian()
        
        # Innovation
        z_pred = self.predict_imu_measurement()
        y = z - z_pred
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update
        self.P = (np.eye(10) - K @ H) @ self.P
    
    def gps_callback(self, msg):
        """Update step with GPS measurement"""
        # Convert lat/lon to local coordinates
        x_gps, y_gps, z_gps = self.latlon_to_local(
            msg.latitude, msg.longitude, msg.altitude)
        
        z = np.array([x_gps, y_gps, z_gps])
        
        # Measurement model (GPS measures position directly)
        H = np.zeros((3, 10))
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        H[2, 2] = 1  # z
        
        # Innovation
        y = z - H @ self.x
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update
        self.x = self.x + K @ y
        self.P = (np.eye(10) - K @ H) @ self.P
    
    def odom_callback(self, msg):
        """Update step with wheel odometry"""
        v_measured = msg.twist.twist.linear.x
        
        z = np.array([v_measured])
        
        # Measurement model (odometry measures vx)
        H = np.zeros((1, 10))
        H[0, 6] = 1  # vx
        
        # Update (same as above)
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.P = (np.eye(10) - K @ H) @ self.P
    
    def publish_state(self):
        """Publish odometry message"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Position
        msg.pose.pose.position.x = self.x[0]
        msg.pose.pose.position.y = self.x[1]
        msg.pose.pose.position.z = self.x[2]
        
        # Orientation (from roll, pitch, yaw)
        msg.pose.pose.orientation = self.euler_to_quaternion(
            self.x[3], self.x[4], self.x[5])
        
        # Velocity
        msg.twist.twist.linear.x = self.x[6]
        msg.twist.twist.linear.y = self.x[7]
        msg.twist.twist.linear.z = self.x[8]
        msg.twist.twist.angular.z = self.x[9]
        
        # Covariances
        msg.pose.covariance = self.P.flatten().tolist()[:36]
        
        self.pub.publish(msg)
```

#### Action Items

- [ ] Implement EKF prediction step
- [ ] Implement IMU update step
- [ ] Implement GPS update step
- [ ] Implement wheel odometry update
- [ ] Tune process noise (Q matrix)
- [ ] Tune measurement noise (R matrices)
- [ ] Test with simulation data
- [ ] Validate accuracy (<10cm position error)

#### Acceptance Criteria

✅ Position error <10cm RMS  
✅ Heading error <2° RMS  
✅ Runs at 100 Hz stably  
✅ Handles sensor dropouts gracefully  

---

### 🗺️ Task 3.2: SLAM Cone Mapping

**Owner**: Localization Team  
**Duration**: 3 weeks  
**Priority**: 🟡 High

#### Deliverables

```python
# localization/slam/slam_node.py
import rclpy
from rclpy.node import Node
from fs_msgs.msg import ConeArray, Cone, ConeMap
from nav_msgs.msg import Odometry
from scipy.spatial.distance import mahalanobis
import numpy as np

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # Global cone map
        self.cone_map = []  # List of Cone objects in global frame
        
        # Subscribers
        self.cone_sub = self.create_subscription(
            ConeArray, '/perception/cones_fused',
            self.cone_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/localization/odometry',
            self.odom_callback, 10)
        
        # Publisher
        self.map_pub = self.create_publisher(
            ConeMap, '/localization/cone_map', 10)
        
        # Current vehicle pose
        self.current_pose = None
        
        # Parameters
        self.association_threshold = 1.0  # Mahalanobis distance
        self.min_observations = 3  # Minimum sightings to trust cone
        
        # Timer for map publishing
        self.create_timer(0.1, self.publish_map)
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def cone_callback(self, msg):
        """Process new cone observations"""
        if self.current_pose is None:
            return
        
        for observed_cone in msg.cones:
            # Transform to global frame
            global_cone = self.transform_to_global(observed_cone, self.current_pose)
            
            # Data association: find if cone already in map
            match_idx = self.associate_cone(global_cone)
            
            if match_idx is not None:
                # Update existing cone (EKF update)
                self.update_cone(match_idx, global_cone)
            else:
                # Add new cone to map
                self.add_cone(global_cone)
    
    def associate_cone(self, cone):
        """Find matching cone in map using Mahalanobis distance"""
        best_idx = None
        best_dist = self.association_threshold
        
        for idx, map_cone in enumerate(self.cone_map):
            # Same color check
            if map_cone.color != cone.color:
                continue
            
            # Compute Mahalanobis distance
            diff = np.array([
                cone.position.x - map_cone.position.x,
                cone.position.y - map_cone.position.y
            ])
            cov = np.array(cone.covariance).reshape(2, 2)
            dist = mahalanobis(diff, np.zeros(2), np.linalg.inv(cov))
            
            if dist < best_dist:
                best_dist = dist
                best_idx = idx
        
        return best_idx
    
    def update_cone(self, idx, observation):
        """Update cone position using EKF-style fusion"""
        map_cone = self.cone_map[idx]
        
        # Simple averaging (can upgrade to Kalman update)
        n = map_cone.observations
        map_cone.position.x = (map_cone.position.x * n + observation.position.x) / (n + 1)
        map_cone.position.y = (map_cone.position.y * n + observation.position.y) / (n + 1)
        map_cone.observations += 1
        
        # Update confidence
        map_cone.confidence = min(1.0, map_cone.observations / self.min_observations)
    
    def add_cone(self, cone):
        """Add new cone to map"""
        cone.observations = 1
        cone.confidence = 1.0 / self.min_observations
        self.cone_map.append(cone)
    
    def publish_map(self):
        """Publish cone map"""
        msg = ConeMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Only publish cones with sufficient confidence
        msg.cones = [c for c in self.cone_map if c.observations >= self.min_observations]
        
        self.map_pub.publish(msg)
    
    def transform_to_global(self, cone, pose):
        """Transform cone from vehicle frame to global frame"""
        # Extract vehicle position and heading
        x_veh = pose.position.x
        y_veh = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        
        # Rotation matrix
        R = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw),  np.cos(yaw)]
        ])
        
        # Transform
        local_pos = np.array([cone.position.x, cone.position.y])
        global_pos = R @ local_pos + np.array([x_veh, y_veh])
        
        # Create new cone in global frame
        global_cone = Cone()
        global_cone.position.x = global_pos[0]
        global_cone.position.y = global_pos[1]
        global_cone.color = cone.color
        global_cone.covariance = cone.covariance  # TODO: transform covariance
        
        return global_cone
```

#### Action Items

- [ ] Implement data association (nearest neighbor + gating)
- [ ] Implement cone update (Kalman fusion)
- [ ] Add new cone logic
- [ ] Handle loop closure (revisiting start)
- [ ] Implement map pruning (remove outliers)
- [ ] Test with simulated laps
- [ ] Validate map consistency

#### Acceptance Criteria

✅ Map contains >90% of track cones  
✅ <5% false positives  
✅ Position accuracy <0.2m  
✅ Map persists across multiple laps  

---

### 📊 Phase 3 Milestones

| Milestone | Completion Criteria | Target | Status |
|-----------|---------------------|--------|--------|
| M3.1 - EKF Working | Localization <10cm error | Week 3 | 🚧 |
| M3.2 - SLAM Functional | Map builds correctly | Week 5 | ⏳ |
| M3.3 - Integration Test | 5 laps in simulation | Week 7 | ⏳ |
| M3.4 - Phase Complete | All tests passing | Week 8 | ⏳ |

---

## Phase 4: Planning & Trajectory

**Duration**: 2 months (Month 7-8)  
**Status**: ⏳ Not Started  
**Team Focus**: Planning Lead + 2 developers

### 🎯 Objectives

Generate optimal racing trajectories that minimize lap time while respecting vehicle constraints.

---

### 🛤️ Task 4.1: Path Planning (Delaunay)

**Owner**: Planning Lead  
**Duration**: 3 weeks  
**Priority**: 🔴 Critical

#### Algorithm Overview

```
Cone Map → Delaunay Triangulation → Extract Midline → Smooth Path
```

#### Implementation

```python
# planning/path_planning/delaunay_planner.py
import rclpy
from rclpy.node import Node
from fs_msgs.msg import ConeMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial import Delaunay
from scipy.interpolate import splprep, splev
import numpy as np

class DelaunayPlannerNode(Node):
    def __init__(self):
        super().__init__('delaunay_planner')
        
        self.sub = self.create_subscription(
            ConeMap, '/localization/cone_map', self.map_callback, 10)
        self.pub = self.create_publisher(
            Path, '/planning/centerline', 10)
    
    def map_callback(self, msg):
        # Separate cones by color
        blue_cones = [(c.position.x, c.position.y) 
                      for c in msg.cones if c.color == Cone.BLUE]
        yellow_cones = [(c.position.x, c.position.y) 
                        for c in msg.cones if c.color == Cone.YELLOW]
        
        if len(blue_cones) < 3 or len(yellow_cones) < 3:
            return
        
        # Compute Delaunay triangulation
        all_cones = np.array(blue_cones + yellow_cones)
        tri = Delaunay(all_cones)
        
        # Extract midline (edges connecting blue-yellow)
        midpoints = []
        for simplex in tri.simplices:
            # Check if triangle has both blue and yellow vertices
            vertices = all_cones[simplex]
            if self.has_blue_and_yellow(simplex, len(blue_cones)):
                # Compute midpoint of blue-yellow edge
                for i in range(3):
                    j = (i + 1) % 3
                    if self.is_blue_yellow_edge(simplex[i], simplex[j], len(blue_cones)):
                        midpoint = (vertices[i] + vertices[j]) / 2
                        midpoints.append(midpoint)
        
        if len(midpoints) < 5:
            return
        
        # Order midpoints (create path)
        ordered_path = self.order_points(midpoints)
        
        # Smooth with cubic spline
        smooth_path = self.smooth_path(ordered_path)
        
        # Publish
        self.publish_path(smooth_path)
    
    def order_points(self, points):
        """Order points to form a continuous path"""
        # Start from point closest to vehicle
        ordered = [points[0]]
        remaining = list(points[1:])
        
        while remaining:
            # Find nearest point
            last = ordered[-1]
            distances = [np.linalg.norm(last - p) for p in remaining]
            nearest_idx = np.argmin(distances)
            ordered.append(remaining[nearest_idx])
            remaining.pop(nearest_idx)
        
        return np.array(ordered)
    
    def smooth_path(self, points, num_points=100):
        """Smooth path using cubic splines"""
        tck, u = splprep([points[:, 0], points[:, 1]], s=0.5, k=3)
        u_fine = np.linspace(0, 1, num_points)
        smooth = splev(u_fine, tck)
        return np.column_stack(smooth)
```

#### Action Items

- [ ] Implement Delaunay triangulation
- [ ] Extract blue-yellow edges
- [ ] Compute midline
- [ ] Order points into path
- [ ] Smooth with splines
- [ ] Handle track closure (loop)
- [ ] Test with various track layouts

#### Acceptance Criteria

✅ Generates valid path for all tracks  
✅ Path stays within track boundaries  
✅ Smooth (C² continuous)  
✅ Computes in <100ms  

---

### 📈 Task 4.2: Trajectory Optimization

**Owner**: Planning Team  
**Duration**: 3 weeks  
**Priority**: 🔴 Critical

#### Minimum Curvature Optimization

```python
# planning/trajectory/optimizer_node.py
from casadi import *
import numpy as np

class TrajectoryOptimizer:
    def __init__(self, path, vehicle_params):
        self.path = path
        self.N = len(path)
        
        # Vehicle constraints
        self.v_max = vehicle_params['v_max']  # 80 km/h
        self.ay_max = vehicle_params['ay_max']  # 1.5g
        self.ax_max = vehicle_params['ax_max']  # 1.0g
        self.kappa_max = vehicle_params['kappa_max']  # 1/3 m^-1
    
    def optimize(self):
        # Decision variables
        opti = Opti()
        
        # Path position offsets (optimize lateral position)
        offsets = opti.variable(self.N)
        
        # Velocities
        velocities = opti.variable(self.N)
        
        # Objective: minimize lap time
        dt = []
        for i in range(self.N - 1):
            ds = self.path_segment_length(i)
            dt.append(ds / (velocities[i] + 1e-6))
        
        opti.minimize(sum1(vcat(dt)))
        
        # Constraints
        for i in range(self.N):
            # Track boundaries
            opti.subject_to(offsets[i] >= -1.5)  # Left boundary
            opti.subject_to(offsets[i] <= 1.5)   # Right boundary
            
            # Speed limits
            opti.subject_to(velocities[i] >= 1.0)
            opti.subject_to(velocities[i] <= self.v_max)
            
            # Lateral acceleration (from curvature)
            kappa = self.compute_curvature(i, offsets)
            ay = velocities[i]**2 * kappa
            opti.subject_to(ay <= self.ay_max)
        
        # Longitudinal acceleration limits
        for i in range(self.N - 1):
            dv = velocities[i+1] - velocities[i]
            ds = self.path_segment_length(i)
            ax = velocities[i] * dv / ds
            opti.subject_to(ax <= self.ax_max)
            opti.subject_to(ax >= -self.ax_max)
        
        # Solve
        opti.solver('ipopt')
        sol = opti.solve()
        
        # Extract solution
        optimal_offsets = sol.value(offsets)
        optimal_velocities = sol.value(velocities)
        
        return optimal_offsets, optimal_velocities
```

#### Action Items

- [ ] Implement curvature calculation
- [ ] Set up optimization problem (CasADi)
- [ ] Add vehicle constraints
- [ ] Implement velocity profiling
- [ ] Test with different tracks
- [ ] Tune solver parameters
- [ ] Validate against theoretical limits

#### Acceptance Criteria

✅ Generates feasible trajectories  
✅ Respects all constraints  
✅ Near-optimal lap times (within 5% of theoretical)  
✅ Solver converges reliably  

---

### 📊 Phase 4 Milestones

| Milestone | Completion Criteria | Target | Status |
|-----------|---------------------|--------|--------|
| M4.1 - Path Planning | Delaunay working | Week 3 | ⏳ |
| M4.2 - Optimization | Trajectory optimal | Week 5 | ⏳ |
| M4.3 - Velocity Profile | Speeds optimized | Week 7 | ⏳ |
| M4.4 - Phase Complete | All tests passing | Week 8 | ⏳ |

---

## Phase 5: Control System

**Duration**: 2 months (Month 9-10)  
**Status**: ⏳ Not Started (25% design complete)  
**Team Focus**: Control Lead + 2 developers

### 🎯 Objectives

Implement precise trajectory tracking using Model Predictive Control (lateral) and PID (longitudinal).

---

### 🎛️ Task 5.1: Model Predictive Control (MPC)

**Owner**: Control Lead  
**Duration**: 4 weeks  
**Priority**: 🔴 Critical

#### Implementation

```python
# control/mpc/mpc_controller.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fs_msgs.msg import Trajectory
from std_msgs.msg import Float64
import casadi as ca
import numpy as np

class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # MPC parameters
        self.N = 20  # Prediction horizon
        self.dt = 0.05  # 50ms
        self.L = 1.6  # Wheelbase (meters)
        
        # Subscribers
        self.traj_sub = self.create_subscription(
            Trajectory, '/planning/trajectory', self.traj_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/localization/odometry', self.odom_callback, 10)
        
        # Publisher
        self.steer_pub = self.create_publisher(
            Float64, '/control/steering_cmd', 10)
        
        # State
        self.current_state = None
        self.reference_traj = None
        
        # Control loop (20 Hz)
        self.create_timer(0.05, self.control_loop)
        
        # Setup MPC problem
        self.setup_mpc()
    
    def setup_mpc(self):
        """Setup CasADi optimization problem"""
        self.opti = ca.Opti()
        
        # Decision variables
        self.X = self.opti.variable(4, self.N+1)  # [x, y, psi, v]
        self.U = self.opti.variable(1, self.N)    # [delta]
        
        # Parameters (will be set each iteration)
        self.X0 = self.opti.parameter(4)  # Initial state
        self.Xref = self.opti.parameter(4, self.N)  # Reference trajectory
        
        # Cost weights
        Q = np.diag([100, 100, 10, 1])  # State error weights
        R = np.array([[1]])  # Control effort weight
        
        # Objective function
        cost = 0
        for k in range(self.N):
            state_error = self.X[:, k] - self.Xref[:, k]
            cost += ca.mtimes([state_error.T, Q, state_error])
            cost += ca.mtimes([self.U[:, k].T, R, self.U[:, k]])
        
        self.opti.minimize(cost)
        
        # Dynamics constraints
        for k in range(self.N):
            x_next = self.kinematic_model(self.X[:, k], self.U[:, k])
            self.opti.subject_to(self.X[:, k+1] == x_next)
        
        # Initial condition
        self.opti.subject_to(self.X[:, 0] == self.X0)
        
        # Input constraints
        delta_max = np.radians(25)  # ±25° steering
        self.opti.subject_to(self.opti.bounded(-delta_max, self.U, delta_max))
        
        # Solver options
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.opti.solver('ipopt', opts)
    
    def kinematic_model(self, state, control):
        """Bicycle model dynamics"""
        x, y, psi, v = state[0], state[1], state[2], state[3]
        delta = control[0]
        
        x_next = x + v * ca.cos(psi) * self.dt
        y_next = y + v * ca.sin(psi) * self.dt
        psi_next = psi + (v / self.L) * ca.tan(delta) * self.dt
        v_next = v  # Assume constant (controlled separately)
        
        return ca.vertcat(x_next, y_next, psi_next, v_next)
    
    def control_loop(self):
        if self.current_state is None or self.reference_traj is None:
            return
        
        # Extract reference for horizon
        ref = self.get_reference_trajectory()
        
        # Set parameters
        self.opti.set_value(self.X0, self.current_state)
        self.opti.set_value(self.Xref, ref)
        
        try:
            # Solve
            sol = self.opti.solve()
            
            # Extract first control input
            steering_angle = sol.value(self.U[0, 0])
            
            # Publish
            msg = Float64()
            msg.data = float(steering_angle)
            self.steer_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"MPC failed: {e}")
```

#### Action Items

- [ ] Implement kinematic bicycle model
- [ ] Set up CasADi optimization
- [ ] Tune cost weights (Q, R)
- [ ] Add steering rate constraints
- [ ] Implement warm-start (speed up solving)
- [ ] Test with various trajectories
- [ ] Validate tracking accuracy

#### Acceptance Criteria

✅ Lateral error <10cm RMS  
✅ Heading error <2° RMS  
✅ Solver time <40ms (20 Hz control)  
✅ Stable at high speeds (60+ km/h)  

---

### 🚗 Task 5.2: PID Speed Control

**Owner**: Control Team  
**Duration**: 2 weeks  
**Priority**: 🟡 High

#### Implementation

```python
# control/pid/speed_controller.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fs_msgs.msg import Trajectory
from std_msgs.msg import Float64

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller')
        
        # PID gains
        self.Kp = 0.8
        self.Ki = 0.1
        self.Kd = 0.2
        
        # State
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.error_integral = 0.0
        self.last_error = 0.0
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/localization/odometry', self.odom_callback, 10)
        self.traj_sub = self.create_subscription(
            Trajectory, '/planning/trajectory', self.traj_callback, 10)
        
        # Publishers
        self.throttle_pub = self.create_publisher(
            Float64, '/control/throttle_cmd', 10)
        self.brake_pub = self.create_publisher(
            Float64, '/control/brake_cmd', 10)
        
        # Control loop (100 Hz)
        self.dt = 0.01
        self.create_timer(self.dt, self.control_loop)
    
    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x
    
    def traj_callback(self, msg):
        # Get target speed from closest waypoint
        self.target_speed = msg.velocities[0]  # Simplified
    
    def control_loop(self):
        # Compute error
        error = self.target_speed - self.current_speed
        
        # PID terms
        P = self.Kp * error
        self.error_integral += error * self.dt
        self.error_integral = np.clip(self.error_integral, -5.0, 5.0)  # Anti-windup
        I = self.Ki * self.error_integral
        D = self.Kd * (error - self.last_error) / self.dt
        
        # Control signal
        control = P + I + D
        
        # Split into throttle/brake
        if control > 0:
            throttle = min(control, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(-control, 1.0)
        
        # Publish
        self.throttle_pub.publish(Float64(data=throttle))
        self.brake_pub.publish(Float64(data=brake))
        
        self.last_error = error
```

#### Action Items

- [ ] Implement PID controller
- [ ] Tune gains (Kp, Ki, Kd)
- [ ] Add anti-windup logic
- [ ] Implement feed-forward term
- [ ] Test acceleration/deceleration
- [ ] Handle emergency braking

#### Acceptance Criteria

✅ Speed tracking error <2 km/h  
✅ Smooth acceleration (no jerking)  
✅ Quick response (<1s to target)  
✅ Emergency stop <50m from 80 km/h  

---

### 📊 Phase 5 Milestones

| Milestone | Completion Criteria | Target | Status |
|-----------|---------------------|--------|--------|
| M5.1 - MPC Working | Tracking error <10cm | Week 4 | ⏳ |
| M5.2 - Speed Control | PID tuned, tracking good | Week 6 | ⏳ |
| M5.3 - Safety Systems | E-stop, limits working | Week 7 | ⏳ |
| M5.4 - Phase Complete | Closed-loop autonomous lap | Week 8 | ⏳ |

---

## Phase 6: Integration & Testing

**Duration**: 1 month (Month 11)  
**Status**: ⏳ Not Started  
**Team Focus**: All team members

### 🎯 Objectives

Integrate all modules, test end-to-end performance, and prepare for hardware deployment.

---

### 🔗 Task 6.1: Full System Integration

**Owner**: System Architect  
**Duration**: 2 weeks  
**Priority**: 🔴 Critical

#### Action Items

- [ ] Create master launch file (all nodes)
- [ ] Verify all topic connections
- [ ] Test full pipeline in simulation
- [ ] Profile CPU/memory usage
- [ ] Optimize bottlenecks
- [ ] Document system parameters
- [ ] Create health monitoring dashboard

---

### ✅ Task 6.2: Validation & Testing

**Owner**: Testing Lead  
**Duration**: 2 weeks  
**Priority**: 🔴 Critical

#### Test Scenarios

```yaml
Simulation Tests:
  1. Simple Oval:
     - 10 consecutive laps
     - Target: <5% lap time variance
  
  2. Complex Track:
     - Tight turns, chicanes
     - Target: No DNF, stay in bounds
  
  3. Edge Cases:
     - Missing cones
     - GPS dropouts
     - Camera occlusions
  
  4. Stress Tests:
     - 100 laps continuous
     - Different lighting/weather
```

#### Action Items

- [ ] Define test scenarios
- [ ] Run simulation tests
- [ ] Log all data (ros2 bag)
- [ ] Analyze results
- [ ] Fix critical bugs
- [ ] Re-test after fixes
- [ ] Document known limitations

---

## Phase 7: Competition Preparation

**Duration**: 1 month (Month 12)  
**Status**: ⏳ Not Started  
**Team Focus**: All team members

### 🏁 Competition Readiness

#### 📋 Task 7.1: Documentation

- [ ] Design report (engineering justification)
- [ ] Safety documentation
- [ ] Scrutineering forms
- [ ] System architecture diagrams
- [ ] Source code documentation

#### 🛡️ Task 7.2: Safety Validation

- [ ] Test all E-stop mechanisms
- [ ] Validate watchdog timers
- [ ] Emergency brake tests
- [ ] Sensor failure recovery
- [ ] Remote kill switch

#### 🏎️ Task 7.3: Competition Events

- [ ] Practice Trackdrive (simulation)
- [ ] Practice Autocross
- [ ] Practice Skidpad
- [ ] Practice Acceleration
- [ ] Operator training

---

## 📚 Resources & Best Practices

### Learning Resources

**ROS 2**
- Official Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- The Construct: https://www.theconstructsim.com/

**Computer Vision**
- YOLOv8 Docs: https://docs.ultralytics.com/
- OpenCV Tutorials: https://docs.opencv.org/

**Control Theory**
- MPC Course: https://www.do-mpc.com/
- Coursera: Vehicle Dynamics & Control

**Formula Student**
- FSG Rules: https://www.formulastudent.de/fsg/rules/
- AMZ Driverless: https://github.com/AMZ-Driverless

### Development Best Practices

```yaml
Code Quality:
  - Write unit tests (pytest, gtest)
  - Use linters (pylint, clang-tidy)
  - Follow style guides (PEP 8, Google C++)
  - Document all functions (docstrings)

Version Control:
  - Feature branches
  - Pull request reviews
  - Semantic commit messages
  - Tag releases (v1.0, v1.1, etc.)

Testing:
  - Test early, test often
  - Automate regression tests
  - Simulate edge cases
  - Log everything

Collaboration:
  - Daily standups
  - Weekly sprint reviews
  - Share knowledge (wiki)
  - Pair programming for complex tasks
```

---

## 🎉 Success Criteria Summary

### System-Level Requirements

| Requirement | Target | Status |
|-------------|--------|--------|
| End-to-end latency | <200ms | ⏳ |
| Cone detection rate | >95% | 🚧 70% |
| Localization error | <10cm RMS | 🚧 55% |
| Lateral tracking error | <10cm RMS | ⏳ |
| Speed tracking error | <2 km/h | ⏳ |
| Autonomous laps (sim) | 10 consecutive | ⏳ |
| Competition events | All 4 completed | ⏳ |

---

<div align="center">

## 🚀 Let's Build Something Amazing!

This roadmap is a living document. Update it as we progress, learn, and adapt. Every line of code, every test, every debug session brings us closer to the podium.

**Formula Student Autonomous Team - 2026**

---

[⬅️ Back to Main README](../README.md) | [System Architecture](architecture.md) | [System Overview](system_overview.md)

*Document Version 1.0 | Last Updated: February 2026*

</div>

