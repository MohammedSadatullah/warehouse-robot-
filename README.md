# Delivery Robot Sim

ROS 2 Jazzy package skeleton for a GPS-free last-mile delivery robot project.

## Current contents

- `worlds/warehouse.sdf`: custom warehouse world with symmetric aisles and dynamic actors
- `launch/warehouse_world.launch.py`: Gazebo Harmonic launch for the warehouse world
- `urdf/delivery_robot.urdf.xacro`: placeholder robot model
- `config/slam_toolbox_params.yaml`: bootstrap SLAM Toolbox parameters
- `config/nav2_params.yaml`: bootstrap Nav2 parameters
- `scripts/goal_runner.py`: placeholder experiment runner

## Package layout

```text
delivery_robot_sim/
|-- CMakeLists.txt
|-- package.xml
|-- README.md
|-- WAREHOUSE_README.md
|-- config/
|-- launch/
|-- scripts/
|-- urdf/
`-- worlds/
```

## Ubuntu Jazzy bring-up

From your ROS 2 workspace:

```bash
colcon build --packages-select delivery_robot_sim
source install/setup.bash
ros2 launch delivery_robot_sim warehouse_world.launch.py
```

## Planned next steps

1. Replace the placeholder URDF with a differential-drive robot and 2D LiDAR.
2. Spawn the robot into Gazebo and bridge `/scan`, `/tf`, and odometry topics.
3. Add SLAM Toolbox online mapping and Nav2 navigation launch files.
4. Implement automated trials and metrics logging for the paper.
