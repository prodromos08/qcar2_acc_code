# QCar2 ACC 2026 – Virtual Stage Autonomous Taxi System  
University of Cyprus – DNCS Lab  

---

## Overview

This repository contains the full development stack for the Quanser QCar2 ACC 2026 Virtual Stage Competition.

The goal is to implement a fully autonomous taxi mission in QLabs demonstrating:

- Mapping
- Localization
- Navigation (Nav2)
- Lane safety
- Traffic sign compliance
- Mission state logic
- Command arbitration
- Hardware interfacing

The system runs entirely in the Quanser QLabs virtual environment using ROS 2.

---

## System Architecture

The stack is divided into modular components.

### Localization
- Static SDCS map
- AMCL localization
- Required TF chain:

map → odom → base_link

If this TF chain does not exist, navigation will fail.

---

### Navigation
- Nav2 (Planner + MPPI Controller)
- Waypoint follower
- Pure pursuit local path following
- Goal sending via RViz or CLI

---

### Lane & Safety Control
- Lane PD controller
- Nav2 velocity output
- `cmd_mux` for arbitration
- `motion_gate` safety gating

Only one node publishes to `/qcar2_motor_speed_cmd`.

---

### Hardware Interface
- `nav2_qcar2_command_converter`
- `qcar2_hardware`

These convert `/cmd_vel_nav` to actual steering and throttle commands.

---

### Mission Logic (Taxi State Machine)
- `trip_planner.py`

Taxi flow:
1. Start at Taxi Hub (LED Magenta)
2. Navigate to Pickup
3. Stop → LED Blue
4. Navigate to Dropoff
5. Stop → LED Orange
6. Return to Hub → LED Magenta

---

### Perception
- YOLO-based traffic detection
- Stop / Yield logic
- Traffic light interpretation
- RGB + depth processing

Perception publishes `/motion_enable` to allow or stop motion.

---

## Virtual Setup

Two containers are used.

---

### Container A – Quanser Virtual Environment

Start the virtual world:

docker run --rm -it --network host --name virtual-qcar2 quanser/virtual-qcar2 bash

Spawn the competition map:

cd /home/qcar2_scripts/python
python3 Base_Scenarios_Python/Setup_Competition_Map.py

This spawns:
- Competition map
- QCar2 at Taxi Hub

---

### Container B – Isaac ROS Development

From host:

cd ~/Documents/ACC_Development/isaac_ros_common
./scripts/run_dev.sh ~/Documents/ACC_Development/Development

All ROS builds and launches run inside this container.

---

## Bringup Procedure

### 1. Start QLabs Map (Container A)

### 2. Launch Navigation Stack (Container B)

ros2 launch qcar2_nodes qcar2_slam_and_nav_bringup_virtual_launch.py

If map server is not publishing:

ros2 lifecycle set /sdcs_map_server configure
ros2 lifecycle set /sdcs_map_server activate

---

### 3. Verify TF

ros2 run tf2_ros tf2_echo map base_link

You MUST see:

map → odom → base_link

If not, navigation will never work.

---

### 4. Send a Goal

In RViz:
- Fixed Frame = map
- Use 2D Pose Estimate
- Use 2D Goal Pose

Or CLI:

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...

---

## Costmap Tuning

Initial parameters:
robot_radius = 0.3
inflation_radius = 0.1

This was too large for the narrow lanes.

Recommended:
robot_radius ≈ 0.18–0.22
inflation_radius ≈ 0.6

If `/map` contains mostly -1 (unknown values), lane enforcement will not work properly.

---

## Common Issues Encountered

- Missing TF frames (map → odom)
- Duplicate lifecycle nodes
- Lifecycle stuck in "unconfigured"
- Map publishing unknown cells (-1)
- Sim time mismatch when `/clock` not present
- YOLO class misalignment causing incorrect stop detections
- Nav2 send_goal hanging due to inactive action server

---

## Debug Reset

Kill stale processes:

pkill -9 -f "nav2|amcl|map_server"
ros2 daemon stop
rm -rf ~/.ros/ros2daemon*
ros2 daemon start

Check system state:

ros2 topic list
ros2 node list
ros2 lifecycle get /amcl
ros2 topic echo /map --once

Ensure `/amcl_pose` publisher count is 1.

---

## Current Status

Working:
- Static map localization
- AMCL
- Nav2 goal navigation
- Command conversion to QCar2 motor interface
- Waypoint following
- Basic lane control
- Taxi mission structure

In Progress:
- Improved costmap corridor stability
- ODESSA depth-based adaptive cruise control
- Traffic sign integration refinement
- Full autonomous taxi demonstration run

---

## Competition Objective

Deliver a stable autonomous taxi mission in QLabs demonstrating:

- Reliable localization
- Safe navigation within lane corridors
- Compliance with traffic rules
- Correct LED mission signaling
- Clear demonstration of core autonomous system principles

---
