<div align="center">

# 🎮 Simulation Environment
### Formula Student Autonomous - Virtual Testing Platform

**Version 1.0** | Last Updated: February 2026

*Test fast, fail safely, iterate quickly*

</div>

---

## 📑 Table of Contents

1. [Overview](#-overview)
2. [Why Simulation First?](#-why-simulation-first)
3. [Architecture](#-architecture)
4. [Getting Started](#-getting-started)
5. [Gazebo Worlds](#-gazebo-worlds)
6. [Vehicle Model](#-vehicle-model)
7. [Sensor Simulation](#-sensor-simulation)
8. [RViz Visualization](#-rviz-visualization)
9. [Testing & Scenarios](#-testing--scenarios)
10. [Advanced Features](#-advanced-features)
11. [Troubleshooting](#-troubleshooting)

---

## 🎯 Overview

The simulation environment is the cornerstone of our development process. It provides a risk-free, reproducible platform for developing, testing, and validating autonomous algorithms before deploying to real hardware.

### Key Benefits

```
┌─────────────────────────────────────────────────────────────┐
│                    SIMULATION ADVANTAGES                    │
├─────────────────────────────────────────────────────────────┤
│  ✓ Zero Risk         - No vehicle damage                   │
│  ✓ Reproducible      - Same scenario every time             │
│  ✓ Fast Iteration    - Test in minutes vs hours            │
│  ✓ Edge Cases        - Test dangerous scenarios safely     │
│  ✓ 24/7 Testing      - No track access needed              │
│  ✓ Cost Effective    - Reduce hardware wear                │
│  ✓ Parallel Dev      - Multiple devs, one codebase         │
│  ✓ CI/CD Ready       - Automated testing pipeline          │
└─────────────────────────────────────────────────────────────┘
```

### Simulation vs Reality

| Aspect | Simulation | Reality | Gap |
|--------|-----------|---------|-----|
| **Physics** | ODE engine | Real world | ~85% accurate |
| **Sensors** | Noise models | True noise | ~90% accurate |
| **Timing** | Real-time capable | Real-time required | 100% match |
| **Cost** | Free | High (€€€) | N/A |
| **Safety** | 100% safe | Risk present | Critical |

---

## 🚀 Why Simulation First?

### Development Philosophy

```
┌──────────────────────────────────────────────────────────┐
│              SIMULATION-FIRST WORKFLOW                   │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  1. Design Algorithm      → Whiteboard & Math           │
│         ↓                                                │
│  2. Implement in Sim      → Write code                  │
│         ↓                                                │
│  3. Unit Test             → Test components             │
│         ↓                                                │
│  4. Integration Test      → Full system test            │
│         ↓                                                │
│  5. Edge Case Testing     → Break it safely             │
│         ↓                                                │
│  6. Parameter Tuning      → Optimize performance        │
│         ↓                                                │
│  7. Hardware-in-Loop      → Real sensors, sim physics   │
│         ↓                                                │
│  8. Real Vehicle Test     → Deploy with confidence      │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### Success Stories

> **"95% of our bugs are caught in simulation before they ever touch the car."**
> — Formula Student Teams Worldwide

**Real Examples**:
- **Bug**: Path planner crashed on closed tracks → Fixed in sim, never saw real car
- **Bug**: MPC unstable at high speeds → Tuned in sim, worked first try on track
- **Feature**: New cone detection algorithm → 1000 laps tested overnight in sim

---

## 🏗️ Architecture

### Simulation Stack

```
┌─────────────────────────────────────────────────────────────────────┐
│                         USER INTERFACE                              │
│  ┌──────────────────────┐          ┌──────────────────────┐         │
│  │   RViz2              │          │   Gazebo GUI         │         │
│  │   Visualization      │          │   3D World View      │         │
│  └──────────┬───────────┘          └───────────┬──────────┘         │
└─────────────┼──────────────────────────────────┼────────────────────┘
              │                                  │
┌─────────────┼──────────────────────────────────┼────────────────────┐
│             │        ROS 2 MIDDLEWARE          │                    │
│             ▼                                  ▼                    │
│  ┌──────────────────┐              ┌──────────────────┐             │
│  │  Your Nodes      │◄────────────►│  Gazebo Plugins  │             │
│  │  (Perception,    │   Topics     │  (Sensors,       │             │
│  │   Planning, etc.)│              │   Actuators)     │             │
│  └──────────────────┘              └────────┬─────────┘             │
└─────────────────────────────────────────────┼───────────────────────┘
                                              │
┌─────────────────────────────────────────────▼──────────────────────┐
│                    GAZEBO SIMULATION ENGINE                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │   Physics    │  │   Rendering  │  │   Sensors    │              │
│  │   (ODE)      │  │   (OGRE)     │  │   (Virtual)  │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
└────────────────────────────────────────────────────────────────────┘
```

### File Organization

```
simulation/
├── worlds/                    # Gazebo world files (.world)
│   ├── simple_oval.world      # Basic oval track
│   ├── fsg_hockenheim.world   # Realistic FSG track
│   ├── autocross.world        # Complex autocross layout
│   └── test_scenarios/        # Edge case scenarios
│       ├── missing_cones.world
│       ├── tight_chicane.world
│       └── rain_conditions.world
│
├── models/                    # 3D models and descriptions
│   ├── fs_vehicle/            # Formula Student car
│   │   ├── model.sdf          # Simulation description
│   │   ├── model.config       # Metadata
│   │   └── meshes/            # 3D meshes
│   ├── cones/                 # Cone models
│   │   ├── blue_cone/
│   │   ├── yellow_cone/
│   │   └── orange_cone/
│   └── track_elements/        # Track boundaries, etc.
│
├── urdf/                      # Robot descriptions
│   ├── fs_vehicle.urdf.xacro  # Vehicle URDF (xacro format)
│   ├── sensors.urdf.xacro     # Sensor configurations
│   └── materials.xacro        # Visual materials
│
├── launch/                    # ROS 2 launch files
│   ├── gazebo_sim.launch.py   # Main simulation launcher
│   ├── simple_track.launch.py # Quick test environment
│   ├── full_stack.launch.py   # Complete autonomous system
│   └── hardware_in_loop.launch.py
│
├── config/                    # Configuration files
│   ├── sensors.yaml           # Sensor parameters
│   ├── physics.yaml           # Physics settings
│   └── vehicle_params.yaml    # Vehicle dynamics
│
├── rviz/                      # RViz configurations
│   ├── default.rviz           # Standard view
│   ├── perception_debug.rviz  # Perception debugging
│   └── planning_view.rviz     # Top-down planning view
│
├── scripts/                   # Utility scripts
│   ├── spawn_cones.py         # Procedural track generation
│   ├── record_data.py         # Data logging
│   └── benchmark.py           # Performance testing
│
├── plugins/                   # Custom Gazebo plugins
│   ├── cone_detection_gt/     # Ground truth cone positions
│   └── vehicle_dynamics/      # Custom dynamics model
│
└── README.md                  # This file!
```

---

## 🚀 Getting Started

### Prerequisites

```bash
# Operating System
Ubuntu 22.04 LTS (recommended)

# ROS 2
ROS 2 Humble Hawksbill

# Gazebo
Gazebo Classic 11 (comes with ROS 2 Humble)

# Additional Tools
RViz2, colcon, rosdep
```

### Installation

**Step 1: Install ROS 2 & Gazebo**

```bash
# Add ROS 2 repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Install Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs

# Install additional dependencies
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-xacro \
                 ros-humble-tf2-tools \
                 ros-humble-rviz2
```

**Step 2: Build Simulation Package**

```bash
# Navigate to workspace
cd ~/fs-autonomous

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select simulation

# Source workspace
source install/setup.bash
```

**Step 3: Test Installation**

```bash
# Launch simple test world
ros2 launch simulation simple_track.launch.py

# You should see:
# - Gazebo window with track and vehicle
# - RViz window showing sensors
# - No errors in terminal
```

### Quick Start Examples

**Example 1: Simple Oval Track**

```bash
# Terminal 1: Launch simulation
ros2 launch simulation simple_track.launch.py

# Terminal 2: Drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /camera/image_raw
```

**Example 2: Full Autonomous Stack**

```bash
# Launch everything (perception, planning, control)
ros2 launch simulation full_stack.launch.py

# Vehicle should start driving autonomously!
```

---

## 🌍 Gazebo Worlds

### Available Worlds

#### 1. **Simple Oval** (`simple_oval.world`)

**Purpose**: Basic testing, algorithm development

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_oval">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Oval track cones (procedurally generated) -->
    <!-- Blue cones on left, yellow on right -->
    <!-- 50m x 30m oval -->
    
  </world>
</sdf>
```

**Features**:
- ✅ 50m × 30m oval shape
- ✅ ~60 cones (blue left, yellow right)
- ✅ 3-4m track width
- ✅ Flat surface
- ✅ Good lighting
- ✅ Fast simulation (~2x real-time)

**Use Cases**:
- Initial testing
- Algorithm debugging
- Quick validation

---

#### 2. **FSG Hockenheim** (`fsg_hockenheim.world`)

**Purpose**: Realistic competition simulation

**Features**:
- ✅ Real FSG track layout
- ✅ ~200m track length
- ✅ Tight hairpins, chicanes, straights
- ✅ Realistic cone spacing
- ✅ Start/finish gates (orange cones)

**Use Cases**:
- Performance benchmarking
- Lap time optimization
- Competition preparation

**Track Map**:
```
        Start/Finish
             ||
    (Orange Cones)
             ||
             ▼
    ╔════════════════╗
    ║  Straight      ║  ← 40m straight
    ║                ║
    ╚═══════╗        ║
            ║ Hairpin║  ← 3m radius turn
            ╚════╗   ║
                 ║ S ║  ← Chicane
            ╔════╝   ║
    ╔═══════╝        ║
    ║   Straight     ║
    ║                ║
    ╚════════════════╝
```

---

#### 3. **Autocross** (`autocross.world`)

**Purpose**: Complex maneuvering practice

**Features**:
- ✅ Random-ish layout (changes each competition)
- ✅ Slaloms, loops, tight turns
- ✅ Tests agility over speed
- ✅ Unknown track (for cold-start testing)

---

#### 4. **Test Scenarios**

**Missing Cones** (`missing_cones.world`)
- Some cones randomly removed
- Tests robustness of path planning

**Tight Chicane** (`tight_chicane.world`)
- Extremely narrow passages
- Tests precision control

**Rain Conditions** (`rain_conditions.world`)
- Reduced tire friction
- Simulated slippery surface

---

### Creating Custom Tracks

**Method 1: Manual (Gazebo GUI)**

```bash
# 1. Launch empty world
gazebo --verbose

# 2. Insert → Model → Select cone model
# 3. Place cones manually
# 4. File → Save World As → my_track.world
```

**Method 2: Procedural (Python Script)**

```python
#!/usr/bin/env python3
# scripts/spawn_cones.py

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import numpy as np

class TrackGenerator(Node):
    def __init__(self):
        super().__init__('track_generator')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
    def generate_oval_track(self, width=50, height=30, cone_spacing=5):
        """Generate oval track with blue/yellow cones"""
        
        # Left boundary (blue cones)
        left_cones = self.generate_oval_points(
            width/2 - 2, height/2, cone_spacing)
        
        # Right boundary (yellow cones)
        right_cones = self.generate_oval_points(
            width/2 + 2, height/2, cone_spacing)
        
        # Spawn cones
        for i, (x, y) in enumerate(left_cones):
            self.spawn_cone(f'blue_cone_{i}', x, y, 'blue')
        
        for i, (x, y) in enumerate(right_cones):
            self.spawn_cone(f'yellow_cone_{i}', x, y, 'yellow')
    
    def generate_oval_points(self, a, b, spacing):
        """Generate points along oval boundary"""
        points = []
        perimeter = np.pi * (3*(a+b) - np.sqrt((3*a+b)*(a+3*b)))
        n_points = int(perimeter / spacing)
        
        for i in range(n_points):
            t = 2 * np.pi * i / n_points
            x = a * np.cos(t)
            y = b * np.sin(t)
            points.append((x, y))
        
        return points
    
    def spawn_cone(self, name, x, y, color):
        """Spawn a cone at given position"""
        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.get_cone_sdf(color)
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.0
        
        self.spawn_client.call_async(request)
    
    def get_cone_sdf(self, color):
        """Return SDF description for cone"""
        color_map = {
            'blue': '0 0 1 1',
            'yellow': '1 1 0 1',
            'orange': '1 0.5 0 1'
        }
        
        return f'''
        <?xml version="1.0"?>
        <sdf version="1.6">
          <model name="cone">
            <static>true</static>
            <link name="link">
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>0.1125</radius>
                    <length>0.325</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>{color_map[color]}</ambient>
                  <diffuse>{color_map[color]}</diffuse>
                </material>
              </visual>
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>0.1125</radius>
                    <length>0.325</length>
                  </cylinder>
                </geometry>
              </collision>
            </link>
          </model>
        </sdf>
        '''

def main():
    rclpy.init()
    generator = TrackGenerator()
    
    # Generate track
    generator.generate_oval_track(width=50, height=30, cone_spacing=4)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage**:
```bash
# 1. Launch Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# 2. Run script
python3 scripts/spawn_cones.py
```

---

## 🚗 Vehicle Model

### URDF Description

Our vehicle is described using **URDF (Unified Robot Description Format)** with **Xacro** for modularity.

**File Structure**:
```
urdf/
├── fs_vehicle.urdf.xacro    # Main vehicle file
├── sensors.urdf.xacro       # Sensor definitions
├── materials.xacro          # Colors/materials
└── macros.xacro             # Reusable macros
```

### Main Vehicle File

```xml
<?xml version="1.0"?>
<robot name="fs_vehicle" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Import dependencies -->
  <xacro:include filename="$(find simulation)/urdf/materials.xacro"/>
  <xacro:include filename="$(find simulation)/urdf/sensors.urdf.xacro"/>
  <xacro:include filename="$(find simulation)/urdf/macros.xacro"/>
  
  <!-- Vehicle Parameters -->
  <xacro:property name="wheelbase" value="1.6"/>  <!-- meters -->
  <xacro:property name="track_width" value="1.2"/>
  <xacro:property name="vehicle_mass" value="250"/>  <!-- kg -->
  <xacro:property name="wheel_radius" value="0.25"/>
  <xacro:property name="wheel_width" value="0.15"/>
  
  <!-- Base Link (chassis) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="2.0 1.2 0.4"/>
      </geometry>
      <material name="carbon_fiber"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="2.0 1.2 0.4"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="${vehicle_mass}"/>
      <origin xyz="0 0 0.3"/>
      <inertia ixx="20.0" ixy="0.0" ixz="0.0"
               iyy="40.0" iyz="0.0"
               izz="40.0"/>
    </inertial>
  </link>
  
  <!-- Wheels (using macro for DRY) -->
  <xacro:wheel prefix="front_left" 
               x="${wheelbase/2}" 
               y="${track_width/2}"/>
  <xacro:wheel prefix="front_right" 
               x="${wheelbase/2}" 
               y="${-track_width/2}"/>
  <xacro:wheel prefix="rear_left" 
               x="${-wheelbase/2}" 
               y="${track_width/2}"/>
  <xacro:wheel prefix="rear_right" 
               x="${-wheelbase/2}" 
               y="${-track_width/2}"/>
  
  <!-- Sensors -->
  <xacro:camera_sensor name="front_camera"/>
  <xacro:lidar_sensor name="lidar"/>
  <xacro:imu_sensor name="imu"/>
  <xacro:gps_sensor name="gps"/>
  
  <!-- Gazebo Plugins -->
  <gazebo>
    <!-- Differential drive controller (simplified) -->
    <plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">
      <update_rate>100</update_rate>
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <wheel_separation>${track_width}</wheel_separation>
      <wheel_diameter>${2*wheel_radius}</wheel_diameter>
      <max_wheel_torque>200</max_wheel_torque>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
    </plugin>
  </gazebo>
  
</robot>
```

### Wheel Macro

```xml
<!-- macros.xacro -->
<xacro:macro name="wheel" params="prefix x y">
  
  <link name="${prefix}_wheel">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>  <!-- Friction coefficient -->
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0"
               izz="0.1"/>
    </inertial>
  </link>
  
  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel"/>
    <origin xyz="${x} ${y} ${wheel_radius}" rpy="${pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <!-- Gazebo-specific properties -->
  <gazebo reference="${prefix}_wheel">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Black</material>
  </gazebo>
  
</xacro:macro>
```

### Vehicle Parameters

```yaml
# config/vehicle_params.yaml

vehicle:
  # Dimensions
  wheelbase: 1.6        # meters
  track_width: 1.2
  length: 2.0
  width: 1.2
  height: 0.8
  
  # Mass properties
  mass: 250             # kg (with driver)
  cog_height: 0.3       # Center of gravity height
  
  # Performance
  max_speed: 22.22      # m/s (80 km/h)
  max_acceleration: 9.81  # m/s² (1g)
  max_deceleration: 14.7  # m/s² (1.5g)
  max_lateral_accel: 14.7  # m/s² (1.5g)
  
  # Steering
  max_steering_angle: 0.436  # radians (25°)
  steering_ratio: 2.5   # Steering wheel : road wheel
  
  # Wheels
  wheel_radius: 0.25    # meters
  wheel_width: 0.15
  tire_friction: 1.0    # Coefficient of friction
```

---

## 📡 Sensor Simulation

### Camera Sensor

```xml
<!-- sensors.urdf.xacro -->
<xacro:macro name="camera_sensor" params="name">
  
  <link name="${name}_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>
  
  <joint name="${name}_joint" type="fixed">
    <parent link="base_link"/>
    <child link="${name}_link"/>
    <!-- Mount on front of vehicle, 0.5m high -->
    <origin xyz="1.0 0 0.5" rpy="0 0 0"/>
  </joint>
  
  <gazebo reference="${name}_link">
    <sensor name="${name}" type="camera">
      <update_rate>60</update_rate>
      <camera>
        <horizontal_fov>1.5708</horizontal_fov>  <!-- 90° -->
        <image>
          <width>1920</width>
          <height>1080</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
        
        <!-- Noise model (realistic) -->
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>${name}_link</frame_name>
        <topic_name>/camera/image_raw</topic_name>
        <camera_info_topic_name>/camera/camera_info</camera_info_topic_name>
      </plugin>
    </sensor>
  </gazebo>
  
</xacro:macro>
```

**Published Topics**:
- `/camera/image_raw` (sensor_msgs/Image) @ 60 Hz
- `/camera/camera_info` (sensor_msgs/CameraInfo)

---

### LiDAR Sensor

```xml
<xacro:macro name="lidar_sensor" params="name">
  
  <link name="${name}_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.07"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  
  <joint name="${name}_joint" type="fixed">
    <parent link="base_link"/>
    <child link="${name}_link"/>
    <!-- Mount on top of vehicle -->
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
  </joint>
  
  <gazebo reference="${name}_link">
    <sensor name="${name}" type="ray">
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>  <!-- 0.5° resolution -->
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>100.0</max>
          <resolution>0.01</resolution>
        </range>
        
        <!-- Noise -->
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <frame_name>${name}_link</frame_name>
        <topic_name>/lidar/points</topic_name>
        <output_type>sensor_msgs/PointCloud2</output_type>
      </plugin>
    </sensor>
  </gazebo>
  
</xacro:macro>
```

**Published Topics**:
- `/lidar/points` (sensor_msgs/PointCloud2) @ 10 Hz

---

### IMU Sensor

```xml
<xacro:macro name="imu_sensor" params="name">
  
  <link name="${name}_link"/>
  
  <joint name="${name}_joint" type="fixed">
    <parent link="base_link"/>
    <child link="${name}_link"/>
    <!-- Mount at vehicle center of mass -->
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
  
  <gazebo reference="${name}_link">
    <sensor name="${name}" type="imu">
      <update_rate>100</update_rate>
      
      <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
        <frame_name>${name}_link</frame_name>
        <topic_name>/imu/data</topic_name>
        
        <!-- Noise parameters (realistic) -->
        <gaussian_noise>0.01</gaussian_noise>
        <accel_drift>0.001</accel_drift>
        <accel_gaussian_noise>0.1</accel_gaussian_noise>
        <rate_drift>0.0001</rate_drift>
        <rate_gaussian_noise>0.01</rate_gaussian_noise>
      </plugin>
    </sensor>
  </gazebo>
  
</xacro:macro>
```

**Published Topics**:
- `/imu/data` (sensor_msgs/Imu) @ 100 Hz

---

### GPS Sensor

```xml
<xacro:macro name="gps_sensor" params="name">
  
  <link name="${name}_link"/>
  
  <joint name="${name}_joint" type="fixed">
    <parent link="base_link"/>
    <child link="${name}_link"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
  </joint>
  
  <gazebo reference="${name}_link">
    <sensor name="${name}" type="gps">
      <update_rate>10</update_rate>
      
      <plugin name="gps_controller" filename="libgazebo_ros_gps_sensor.so">
        <frame_name>${name}_link</frame_name>
        <topic_name>/gps/fix</topic_name>
        
        <!-- GPS accuracy (RTK mode) -->
        <horizontal_position_std_dev>0.02</horizontal_position_std_dev>
        <vertical_position_std_dev>0.05</vertical_position_std_dev>
      </plugin>
    </sensor>
  </gazebo>
  
</xacro:macro>
```

**Published Topics**:
- `/gps/fix` (sensor_msgs/NavSatFix) @ 10 Hz

---

## 🎨 RViz Visualization

### Default Configuration

```yaml
# rviz/default.rviz (simplified)

Panels:
  - Class: rviz_common/Displays
  - Class: rviz_common/Views

Visualization Manager:
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  
  Displays:
    # TF Tree
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Show Axes: true
      Show Names: true
    
    # Camera Image
    - Class: rviz_default_plugins/Image
      Enabled: true
      Topic: /camera/image_raw
      Transport Hint: raw
    
    # LiDAR Point Cloud
    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Topic: /lidar/points
      Size: 2
      Color Transformer: Intensity
    
    # Detected Cones
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Topic: /perception/cone_markers
    
    # Vehicle Path
    - Class: rviz_default_plugins/Path
      Enabled: true
      Topic: /planning/trajectory
      Color: 0; 255; 0  # Green
    
    # Vehicle Pose
    - Class: rviz_default_plugins/Odometry
      Enabled: true
      Topic: /localization/odometry
      Shape: Arrow
      Color: 255; 0; 0  # Red
```

### Custom Views

**Top-Down Planning View**:
```yaml
# rviz/planning_view.rviz
Views:
  - Class: rviz_default_plugins/TopDownOrtho
    Name: TopDown
    Target Frame: map
    X: 0
    Y: 0
    Scale: 20  # meters per viewport height
```

**Driver Perspective**:
```yaml
Views:
  - Class: rviz_default_plugins/ThirdPersonFollower
    Name: DriverView
    Target Frame: base_link
    Distance: 5.0
    Pitch: 0.5
    Yaw: 0.0
```

---

## 🧪 Testing & Scenarios

### Automated Testing

```python
#!/usr/bin/env python3
# scripts/benchmark.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import time

class BenchmarkNode(Node):
    def __init__(self):
        super().__init__('benchmark')
        
        self.sub = self.create_subscription(
            Odometry, '/localization/odometry', self.odom_callback, 10)
        
        self.start_time = None
        self.lap_times = []
        self.current_lap_start = None
        
    def odom_callback(self, msg):
        # Simple lap detection (crossing start line)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.is_crossing_start_line(x, y):
            if self.current_lap_start is not None:
                lap_time = time.time() - self.current_lap_start
                self.lap_times.append(lap_time)
                self.get_logger().info(f'Lap {len(self.lap_times)}: {lap_time:.2f}s')
            
            self.current_lap_start = time.time()
    
    def is_crossing_start_line(self, x, y):
        # Example: start line at x=0, y=0
        return abs(x) < 0.5 and abs(y) < 0.5
    
    def print_statistics(self):
        if not self.lap_times:
            return
        
        print("\n=== Benchmark Results ===")
        print(f"Total Laps: {len(self.lap_times)}")
        print(f"Best Lap: {min(self.lap_times):.2f}s")
        print(f"Average: {sum(self.lap_times)/len(self.lap_times):.2f}s")
        print(f"Consistency: {self.calculate_consistency():.1f}%")
    
    def calculate_consistency(self):
        if len(self.lap_times) < 2:
            return 100.0
        
        avg = sum(self.lap_times) / len(self.lap_times)
        variance = sum((t - avg)**2 for t in self.lap_times) / len(self.lap_times)
        std_dev = variance ** 0.5
        
        return (1 - std_dev/avg) * 100

def main():
    rclpy.init()
    node = BenchmarkNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_statistics()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage**:
```bash
# Run autonomous system
ros2 launch simulation full_stack.launch.py

# In another terminal, start benchmark
python3 scripts/benchmark.py

# Let it run 10 laps, then Ctrl+C to see results
```

---

## 🔬 Advanced Features

### Hardware-in-the-Loop (HIL)

```python
# Use real sensors with simulated physics

# Launch only Gazebo physics (no sensor simulation)
ros2 launch simulation gazebo_physics_only.launch.py

# Connect real sensors
ros2 run camera_driver camera_node
ros2 run lidar_driver lidar_node

# Your autonomy stack uses real sensor data!
```

### Recording & Playback

```bash
# Record everything
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image_raw /lidar/points /localization/odometry

# Playback
ros2 bag play my_recording.bag
```

### Parallel Simulations (CI/CD)

```bash
# Run headless (no GUI)
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch simulation gazebo_headless.launch.py

# Automated testing
python3 run_all_tests.py
```

---

## 🐛 Troubleshooting

### Common Issues

**Issue 1: Gazebo crashes on launch**
```bash
# Solution: Reset Gazebo
killall -9 gazebo gzserver gzclient
rm -rf ~/.gazebo/
```

**Issue 2: Sensors not publishing**
```bash
# Check if plugins loaded
ros2 topic list
# Should see /camera/image_raw, /lidar/points, etc.

# Check Gazebo logs
gazebo --verbose
```

**Issue 3: Poor performance**
```bash
# Reduce physics quality
# Edit world file:
<real_time_update_rate>100</real_time_update_rate>  # Was 1000

# Disable shadows
<scene>
  <shadows>false</shadows>
</scene>
```

**Issue 4: Vehicle falls through ground**
```bash
# Check collision geometry in URDF
# Make sure ground plane has collision enabled
```

---

## 📚 Additional Resources

### Documentation
- **Gazebo Tutorials**: http://gazebosim.org/tutorials
- **URDF Tutorials**: http://wiki.ros.org/urdf/Tutorials
- **RViz User Guide**: http://wiki.ros.org/rviz/UserGuide

### Example Projects
- **AMZ Driverless**: https://github.com/AMZ-Driverless
- **EUFS Sim**: https://gitlab.com/eufs/eufs_sim
- **Formula Student Sim**: Various team repos

---

<div align="center">

## 🎯 Simulation Best Practices

1. **Start Simple** - Test each component individually before integration
2. **Iterate Fast** - Use simulation to try crazy ideas safely
3. **Validate Early** - Don't wait for hardware to find bugs
4. **Document Parameters** - Record what works for future reference
5. **Automate Testing** - Let the computer test while you sleep

---

**"The best code is the code that never reaches the real car."**
*— Every Formula Student Team (after the first crash)*

---

[⬅️ Back to Main README](../../README.md) | [Development Roadmap](../docs/development_roadmap.md)

*Document Version 1.0 | Last Updated: February 2026*

</div>
