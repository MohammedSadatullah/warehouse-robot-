# Delivery Robot Simulation

`delivery_robot_sim` is a ROS 2 Jazzy simulation package for a GPS-free warehouse delivery robot workflow. The repository contains a custom Gazebo Harmonic warehouse world, a differential-drive robot description, SLAM bring-up files, and helper scripts for mapping and repeatable navigation experiments.

## Repository Highlights

- Custom warehouse world in `worlds/warehouse_v2.sdf`
- Gazebo launch in `launch/warehouse_world.launch.py`
- Robot + SLAM bring-up in `launch/robot_bringup.launch.py`
- Robot model in `urdf/delivery_robot.urdf.xacro`
- Mapping and experiment helpers in `scripts/`
- Example saved map output in `warehouse_map.yaml` and `warehouse_map.pgm`

## Project Structure

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
|-- worlds/
|-- warehouse_map.pgm
`-- warehouse_map.yaml
```

## Requirements

- Ubuntu with ROS 2 Jazzy
- Gazebo Harmonic
- `slam_toolbox`
- `ros_gz_bridge`
- `robot_state_publisher`
- `rviz2`
- `xacro`

Optional:

- `python3-gz-transport13` for scripted pedestrian motion in `dynamic_actors.py`

## Build

From your ROS 2 workspace:

```bash
colcon build --packages-select delivery_robot_sim
source install/setup.bash
```

## Running The Simulation

Launch the warehouse world:

```bash
ros2 launch delivery_robot_sim warehouse_world.launch.py
```

Launch the robot state publisher, ROS/Gazebo bridge, SLAM Toolbox, and RViz:

```bash
ros2 launch delivery_robot_sim robot_bringup.launch.py
```

## Mapping Scripts

The repository includes a few exploration options in `scripts/`:

- `hardcoded_mapping_v3.py`: odometry-guided waypoint traversal for full warehouse coverage
- `explore_warehouse.py`: reactive waypoint exploration with simple obstacle avoidance
- `dynamic_actors.py`: moves pedestrian obstacles through scripted patrol paths
- `goal_runner.py`: placeholder for repeatable Nav2 goal experiments

Run the main mapping script:

```bash
python3 scripts/hardcoded_mapping_v3.py
```

Save the resulting map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_map \
  --ros-args -p save_map_timeout:=5000.0
```

## Notes

- `WAREHOUSE_README.md` documents the world layout, symmetry rationale, and benchmark goals.
- The repository currently tracks generated map artifacts for convenience; if you prefer a lighter repo, those can be regenerated and removed later.
- The package name is `delivery_robot_sim` even though the local folder name is `mar`.

## Ownership

For repository access, add `mars.ciot@pes.edu` as an admin or contributor in the GitHub repository settings after the remote repository is created.
