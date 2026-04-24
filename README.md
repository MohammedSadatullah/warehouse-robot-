# 🚀 Autonomous Warehouse Robot (ROS 2)

## 📌 Overview
This project implements an autonomous mobile robot for warehouse navigation using ROS 2.  
The robot performs mapping, localization, and navigation in a simulated environment.

---

## 🎯 Objectives
- Perform SLAM-based mapping of the environment  
- Localize the robot using AMCL  
- Navigate to goal positions using Nav2  
- Evaluate performance using metrics  

---

## 🧠 System Pipeline

Gazebo → SLAM → Map → AMCL → Nav2 → Metrics


---

## 🛠️ Technologies Used
- ROS 2 (Jazzy)
- Gazebo Simulator
- RViz
- Python
- Nav2 Stack
- slam_toolbox

---

## ⚙️ Setup Instructions

1. Source ROS 2

source /opt/ros/jazzy/setup.bash

2. Launch Simulation
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

4. Run SLAM (Mapping)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

6. Save Map
ros2 run nav2_map_server map_saver_cli -f ~/final_map

8. Run Navigation (AMCL + Nav2)
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
use_sim_time:=True \
map:=/home/<your-username>/final_map.yaml

🎮 Usage
Open RViz
Click 2D Pose Estimate to initialize robot
Click 2D Goal Pose to send navigation goal
📊 Metrics Evaluation

Metrics logged include:

Localization Error
Travel Time
Success Rate
Replan Count

Output file:

~/metrics_log.csv
🔗 GitHub Repository

https://github.com/MohammedSadatullah/warehouse-robot-

🧠 Conclusion

The project successfully demonstrates autonomous navigation using SLAM and Nav2.
SLAM-based localization proved effective for indoor environments compared to baseline methods.

👨‍💻 Author

Mohammed Sadatullah
