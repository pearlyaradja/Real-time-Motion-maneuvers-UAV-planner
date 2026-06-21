# 3D RRT UAV Motion Planner

A 3D Rapidly-exploring Random Tree (RRT) flight-path planner delivered in two
architectures.

Shared planner specs (identical in both versions):
- Airspace: 100 x 100 x 30 m bounding box
- Step size: 5.0 m max per expansion
- Goal bias: 10%
- Goal-reached threshold: 5.0 m
- Iteration budget: 200
- 3 cuboid obstacles, inflated by a 1.5 m clearance buffer
- Per-node cumulative cost tracking
- Backward parent-trace path extraction

---

## Version 1 - Pure Python

Single self-contained script with matplotlib 3D rendering.

```bash
pip install matplotlib
python rrt_3d_planner.py
```

A figure window opens showing the tree (light blue), obstacles (solid blocks
with a faint inflated shell), Start (green triangle), Goal (red star) and the
final path (thick red line).

Note: vanilla RRT is probabilistically complete, not guaranteed within a fixed
budget. Roughly 85% of random seeds solve inside 200 iterations with this
obstacle layout; on a FAIL just re-run.

---

## Version 2 - Hybrid C++ engine + Python GUI (UDP)

Data flow:
```
[GUI]  --launch subprocess(goal X,Y,Z)-->  [rrt_engine (C++)]
[rrt_engine]  --UDP path string on 127.0.0.1:5005-->  [GUI listener]  --render-->
```

The Python GUI owns (binds) the UDP socket on port 5005 and listens; the C++
engine is the sender. The goal is passed to the engine as command-line args.

### 1. Build the engine

Linux / macOS:
```bash
g++ -std=c++17 -O2 main.cpp -o rrt_engine
```

Windows (MinGW):
```bash
g++ -std=c++17 -O2 main.cpp -o rrt_engine.exe -lws2_32
```

### 2. Run the GUI (keep `rrt_engine` next to `gui_visualizer.py`)

```bash
pip install matplotlib
# Debian/Ubuntu only, if Tkinter is missing:  sudo apt-get install python3-tk
python gui_visualizer.py
```

Type the goal X/Y/Z, click **RUN PLANNER**. The GUI launches the engine, which
solves the path and streams the coordinates back over UDP; the embedded
matplotlib canvas then draws the obstacles and the resolved collision-free path.

### Quick engine test without the GUI

```bash
# terminal 1: listen
python -c "import socket;s=socket.socket(2,2);s.bind(('127.0.0.1',5005));print(s.recvfrom(65535)[0].decode())"
# terminal 2: solve + send
./rrt_engine 90 90 20
```
