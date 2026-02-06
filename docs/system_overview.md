# System Overview

This document describes the high-level architecture of the
Formula Student Autonomous System.

The system is designed to be modular, allowing each subsystem
to be developed, tested and improved independently.

---

## High-Level Architecture

The autonomous system consists of five main modules:

1. Perception
2. Localization
3. Planning
4. Control
5. Simulation

Each module communicates through ROS 2 topics and services.

---

## Data Flow

Sensors provide raw data to the perception module.
Processed information is then used to estimate the vehicle state,
plan a trajectory and generate control commands.

```
Sensors
↓
Perception
↓
Localization
↓
Planning
↓
Control
↓
Vehicle
```

---

## Design Principles

- Modularity
- Scalability
- Real-time performance
- Simulation-first development

---

## Current Focus

- Simulation environment setup
- Basic cone detection
- Simple path planning
