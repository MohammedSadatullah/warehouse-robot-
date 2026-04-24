# Warehouse World Design Notes

## Layout

The warehouse footprint is 20 m x 24 m with a centered south entry gap.
The top-down arrangement is:

```text
W  +--------------------------+  E
E  |  [A][B]  aisle  [D][C]   |  A
S  |  [A][B]  aisle  [D][C]   |  S
T  |  [A][B]  aisle  [D][C]   |  T
   |                          |
   |   workbench (SE corner)  |
   |        [dock]            |
   +-------- gap ------------ +
            SOUTH (entry)
```

## Key dimensions

| Feature | Value | Research rationale |
|---|---:|---|
| Warehouse footprint | 20 x 24 m | Realistic small distribution center |
| Shelf aisle width | 1.1 m | Stresses narrow navigation and replanning |
| Center aisle width | 2.5 m | Main robot transit corridor |
| Shelf height | 1.8 m | Blocks LiDAR line of sight and forces mapping |
| Entry gap | 2.0 m | Simulates a loading bay |
| Robot start | (0, -10) | At the charging dock near the south entrance |

## Why the symmetry matters

Rows A and B on the west side are mirrored by C and D on the east side.
This makes the environment visually similar for a 2D LiDAR and creates a
useful stress case for loop closure and disambiguation in SLAM.

Asymmetric cues are intentionally sparse:

- colored boxes on selected shelves
- the southeast workbench and packing box
- the charging dock near the south entry

This gives you a clean research story:

- successful localization means the robot distinguishes east and west aisles
- failure cases appear as mirrored or drifting poses near row crossings

## Dynamic actors

| Actor | Path | Intended stress case |
|---|---|---|
| `actor_centre` | Center aisle north-south patrol | Head-on interaction in the main corridor |
| `actor_west` | West aisle between rows A and B | Narrow blockage that forces local replanning |
| `actor_cross` | Full east-west crossing at y=0 | Sudden full blockage of the center aisle |

`actor_cross` is the most useful actor for a replanning latency metric because
it cuts across the main corridor abruptly.

## Suggested delivery goals

Use these poses for repeatable Nav2 trials:

| Goal | x | y | theta (rad) | Description |
|---|---:|---:|---:|---|
| G1 | 6.0 | 9.0 | 1.57 | East aisle far end |
| G2 | -6.0 | 9.0 | 1.57 | West aisle far end |
| G3 | 0.0 | 11.0 | 1.57 | North wall |
| G4 | 7.5 | -8.0 | 0.0 | Packing station |
| G5 | -7.5 | 6.0 | 1.57 | West shelf midpoint |

G1 and G2 are the key comparison pair because the environment is nearly
mirrored. A localization system that drifts or over-trusts symmetry may
confuse these goals.

## Expected package structure

```text
delivery_robot_sim/
|-- worlds/
|   `-- warehouse.sdf
|-- launch/
|   `-- warehouse_world.launch.py
|-- urdf/
|   `-- delivery_robot.urdf.xacro
|-- config/
|   |-- slam_toolbox_params.yaml
|   `-- nav2_params.yaml
`-- scripts/
    `-- goal_runner.py
```

## Quick sanity checks in Ubuntu

After launching the world:

```bash
ros2 launch delivery_robot_sim warehouse_world.launch.py
```

In a second terminal:

```bash
ros2 topic echo /clock --once
gz topic -l | grep warehouse
```

Robot-specific checks such as `/scan`, `/tf`, or `/model/delivery_robot/pose`
will be added after the robot model is spawned.
