# Travelling Ethiopia - AI Search Algorithms & Robot Simulation

This project implements various AI search algorithms for pathfinding in Ethiopia's city network and simulates robot navigation using PyBullet.

## Features

### Search Algorithms
- **BFS/DFS** - Uninformed search for finding paths
- **UCS** - Uniform Cost Search for optimal paths
- **A\*** - Heuristic search for faster pathfinding
- **MiniMax** - Adversarial search for coffee quality optimization

### Robot Simulation
- **PyBullet Physics** - 3D simulation environment
- **Differential Drive** - Two-wheeled robot with caster
- **City Visualization** - Interactive map with labeled cities
- **Smooth Navigation** - Animated robot movement

## Project Structure

```
Travelling-Ethiopia/
├── data/                    # Graph data and city coordinates
│   ├── graph1.py          # BFS/DFS graph
│   ├── graph2.py          # UCS/A* graph  
│   ├── graph3.py          # Heuristics
│   ├── graph4.py          # Coffee tree
│   └── graph5.py          # Robot navigation
├── search/                  # Search algorithm implementations
│   ├── bfs_dfs.py        # BFS and DFS
│   ├── ucs.py            # Uniform Cost Search
│   ├── astart.py          # A* Search
│   ├── advs.py            # MiniMax
│   └── robot.py           # Original robot simulation
└── interactive/             # Modular robot simulation
    ├── robot_config.py     # Robot configuration
    ├── robot_factory.py    # Robot creation
    ├── robot_controller.py # Robot control
    ├── bfs_planner.py     # Path planning
    ├── traveller.py        # Main navigation class
    └── __init__.py         # Package initialization
```

## Usage

### Run Original Algorithms
```bash
python3 -m main
```

### Run Interactive Robot Simulation
```bash
python3 -m main  # Uses interactive robot modules
```

## Data Files

- **graph1**: Ethiopian cities network for BFS/DFS
- **graph2**: Weighted graph for UCS/A* with distances
- **graph3**: Heuristic values for A* search
- **graph4**: Coffee quality decision tree for MiniMax
- **graph5**: Simplified graph for robot navigation with coordinates

## Algorithms Implemented

1. **Breadth-First Search** - Finds shortest path in terms of edges
2. **Uniform Cost Search** - Finds optimal path considering edge weights
3. **A\* Search** - Fast optimal path using heuristics
4. **MiniMax** - Game theory for adversarial decision making

## Robot Features

- Three-wheel differential drive robot
- Real-time physics simulation
- City visualization with labels
- Smooth animated movement
- Gyroscope and camera simulation
- Path planning and execution

## Requirements

- Python 3.8+
- PyBullet physics engine
- NumPy for numerical computations
- Standard Python libraries