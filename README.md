# QCar2 ACC 2026 – Virtual Stage Autonomous Taxi System  
University of Cyprus – DNCS Lab  

This repository contains the development stack used for the Quanser QCar2 ACC 2026 Virtual Stage competition. The system runs inside the Quanser QLabs virtual environment using ROS 2.

---

# Full Run Instructions

## 1. Start QLabs Virtual Environment (Terminal A – Host)

docker run --rm -it --network host --name virtual-qcar2 quanser/virtual-qcar2 bash  

Inside the container:

cd /home/qcar2_scripts/python  
python3 Base_Scenarios_Python/Setup_Competition_Map.py  

Leave this terminal running.

---

## 2. Start Isaac ROS Development Container (Terminal B – Host)

cd ~/Documents/ACC_Development/isaac_ros_common  
./scripts/run_dev.sh ~/Documents/ACC_Development/Development  

Inside the container:

source /opt/ros/humble/setup.bash  

If using a workspace:

cd /workspaces/isaac_ros-dev/ros2  
colcon build --symlink-install  
source install/setup.bash  

---

# Lane Following Mode

Open two additional terminals inside the Isaac container.

Terminal C:

cd ~/qcar2_acc_code  
python3 camera_node.py  

Terminal D:

cd ~/qcar2_acc_code  
python3 controller_node.py  

Optional lane debug:

ros2 run rqt_image_view rqt_image_view  

Select topic: /lane_debug  

---

# Autonomous Navigation Mode (Nav2)

Inside the Isaac container:

ros2 launch qcar2_nodes qcar2_slam_and_nav_bringup_virtual_launch.py  

If required:

ros2 lifecycle set /sdcs_map_server configure  
ros2 lifecycle set /sdcs_map_server activate  

Then in another terminal:

cd ~/qcar2_acc_code  
python3 autonomous_driving_node.py  

---

Run either lane mode or Nav2 mode. Do not run both simultaneously.


## Development Status

Due to time constraints and the complexity of integrating multiple subsystems (localization, navigation, perception, and mission logic), a fully stable end-to-end autonomous taxi run was not achieved within the intended timeline.

Core subsystems were developed and tested independently, including:

- Static map localization using AMCL  
- Nav2-based goal navigation  
- Command conversion to QCar2 motor interface  
- Mission state machine structure  
- Basic lane control  

However, full integration revealed instability primarily in:

- Costmap corridor tuning for consistent lane-safe navigation  
- TF synchronization across complete bringup  
- Perception-to-motion gating reliability  
- End-to-end mission stability under repeated runs  

The architecture and modular design are in place, and the remaining effort focuses on integration robustness and system stabilization rather than missing foundational components.

This repository represents a functional research and development stack currently undergoing refinement and validation.
