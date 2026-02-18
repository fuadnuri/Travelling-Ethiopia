# 🇪🇹 Travelling Ethiopia — AI Search Algorithms & Robot Simulation

This project implements various AI search algorithms for pathfinding across Ethiopia's city network and simulates robot navigation using **Gazebo** with ROS 2.

---

## Features

### Search Algorithms
- **BFS / DFS** — Uninformed search for finding paths
- **UCS** — Uniform Cost Search for optimal paths
- **A\*** — Heuristic search for faster pathfinding
- **MiniMax** — Adversarial search for coffee quality optimization

### Robot Simulation
- **Gazebo Physics Engine** — Full 3D simulation environment with ROS 2 (Jazzy)
- **Three-Wheel Differential Drive** — Custom URDF robot model
- **Sensor Suite** — Proximity sensor, gyroscope (IMU), and RGB camera
- **City World** — Ethiopia state-space map as a `.world` file with Cartesian coordinates
- **ROS 2 Navigation** — Path planning and execution via ROS 2 topics

---

## Project Structure

```
Travelling-Ethiopia/
├── data/                        # Graph data and city coordinates
│   ├── graph1.py               # BFS/DFS graph
│   ├── graph2.py               # UCS/A* weighted graph
│   ├── graph3.py               # Heuristics
│   ├── graph4.py               # Coffee tree (MiniMax)
│   └── graph5.py               # Robot navigation graph
├── search/                      # Search algorithm implementations
│   ├── bfs_dfs.py             # BFS and DFS
│   ├── ucs.py                 # Uniform Cost Search
│   ├── astart.py              # A* Search
│   ├── advs.py                # MiniMax
│   └── robot.py               # Robot navigation logic
├── myrobot/                     # ROS 2 workspace
│   └── src/
│       └── my_robot/
│           ├── my_robot/
│           │   ├── ethiopia_search.py   # BFS ROS 2 search node
│           │   └── my_node.py          # Base ROS 2 node
│           ├── urdf/
│           │   └── three_wheel_robot.urdf.xacro  # Robot model
│           ├── worlds/
│           │   └── ethiopia.world      # Gazebo world file
│           ├── setup.py
│           └── package.xml
└── interactive/                 # Modular robot simulation
    ├── robot_config.py         # Robot configuration
    ├── robot_factory.py        # Robot creation
    ├── robot_controller.py     # Robot control
    ├── bfs_planner.py         # Path planning
    ├── traveller.py            # Main navigation class
    └── __init__.py             # Package initialization
```

---

## Preview

### 🗺️ BFS Pathfinder — Interactive Visualizer
<img src="https://github.com/fuadnuri/Travelling-Ethiopia/blob/main/travelling-ethiopia.png">
## Setup & Usage

### Prerequisites
- Ubuntu 22.04+
- ROS 2 Jazzy
- Gazebo (installed via `ros-jazzy-gazebo-ros-pkgs`)
- Python 3.8+

### Install ROS 2 Dependencies
```bash
sudo apt install ros-jazzy-gazebo-ros-pkgs
source /opt/ros/jazzy/setup.zsh   # or setup.bash
```

### Build the ROS 2 Workspace
```bash
cd Travelling-Ethiopia/myrobot
colcon build
source install/setup.zsh
```

### Run Search Algorithms
```bash
python3 -m main
```

### Run BFS Search Node (ROS 2)
```bash
ros2 run my_robot ethiopia_search --ros-args -p start:="Addis Ababa" -p goal:="Moyale"
```

### Launch Gazebo World with Robot
```bash
ros2 launch my_robot ethiopia.launch.py
```

---

## Data Files

| File | Description |
|---|---|
| `graph1.py` | Ethiopian cities network for BFS/DFS |
| `graph2.py` | Weighted graph for UCS/A* with distances |
| `graph3.py` | Heuristic values for A* search |
| `graph4.py` | Coffee quality decision tree for MiniMax |
| `graph5.py` | Simplified graph for robot navigation with coordinates |

---

## Algorithms Implemented

1. **Breadth-First Search** — Finds shortest path in terms of number of edges
2. **Depth-First Search** — Explores paths depth-first
3. **Uniform Cost Search** — Finds optimal path considering edge weights
4. **A\* Search** — Fast optimal path using heuristics
5. **MiniMax** — Game theory for adversarial coffee quality decision making

---

## Robot Features

- Three-wheel differential drive robot (URDF/Xacro)
- Full Gazebo physics simulation (ODE engine)
- Ethiopia city map as a Gazebo `.world` file
- Proximity sensor, IMU (gyroscope), and RGB camera (640×480)
- ROS 2 topic-based path planning and execution
- BFS-powered autonomous navigation between cities

---

## Requirements

```
Python 3.8+
ROS 2 Jazzy
Gazebo (via ros-jazzy-gazebo-ros-pkgs)
NumPy
rclpy
std_msgs
```

Install Python dependencies:
```bash
pip install numpy
```

---

## Authors

Maintained by the **Fuad Nuri**  — AAU AI Principles Project.
