# 3D UAV Motion Planner & Agile Maneuver Space Simulation

A 3D Rapidly-exploring Random Tree (RRT) flight-path planner for Unmanned Aerial Vehicles (UAV) delivered in three different architectures to suit different workflows and platforms.

## 🚀 Shared Planner Specifications
All implementations share the same underlying planner guidelines:
- **Airspace:** Bounded box of $100 \times 100 \times 30$ meters.
- **Inflation Buffer:** Obstacles are inflated by a $1.5$ meter safety clearance buffer.
- **RRT Pathfinding:** Explores airspace dynamically using goal-biased sampling to find a collision-free path.

---

## 📂 Project Architectures & How to Run

Select the version you want to run:

### 1. 🌐 Web Planner (3D WebGL / JavaScript)
A modern, interactive 3D web-based visualization that requires **no installation** and runs directly in the browser. It simulates realistic flight maneuvers using motion primitives (such as steady cruise, sharp turns, knife-edge, climbs, and dives).

#### Features:
- Rendered in real-time WebGL using **Three.js** and **OrbitControls**.
- Supports multiple UAVs path planning.
- Animates tree-growth step-by-step or instantly solves path.
- Interactive 3D controls (left-click to rotate, right-click to pan, scroll to zoom).
- Interactive waypoint listing table.

#### How to Run:
* **Option A: Using Live Server Extension (Recommended)**
  1. Open this project in **VS Code**.
  2. Install the **Live Server** extension (by Ritwick Dey).
  3. Open [web_planner/index.html](file:///d:/UAV%20route%20planning/web_planner/index.html), right-click in the editor, and select **"Open with Live Server"**.
  4. The web planner will launch at `http://127.0.0.1:5500/web_planner/index.html`.

* **Option B: Using Python Local Server**
  1. Open your terminal and navigate to the `web_planner` directory:
     ```bash
     cd web_planner
     ```
  2. Start a local server:
     ```bash
     python -m http.server 8000
     ```
  3. Open your browser and navigate to: [http://localhost:8000](http://localhost:8000)

* **Option C: Direct Double-click**
  1. Navigate to the `web_planner/` folder in your File Explorer.
  2. Double-click [index.html](file:///d:/UAV%20route%20planning/web_planner/index.html) to open it in any browser (Internet connection required to load Three.js CDN libraries).

---

### 2. 🐍 Version 1 - Pure Python
A self-contained Python script with standard 3D Matplotlib visualizer showing the tree expansion, obstacles, and optimal path.

#### Prerequisites:
Make sure you have Python 3.x and `matplotlib` installed:
```bash
pip install matplotlib
```

#### How to Run:
1. Navigate to the `version1_python` folder:
   ```bash
   cd version1_python
   ```
2. Run the script:
   ```bash
   python rrt_3d_planner.py
   ```
   *(A Matplotlib 3D plotting window will open showing the generated RRT path).*

---

## 📚 References

This project is based on the following research paper:

> J. M. Levin, M. Nahon, and A. A. Paranjape, "Real-time motion planning with a fixed-wing UAV using an agile maneuver space," *Autonomous Robots*, vol. 43, no. 8, pp. 2111–2130, Springer Science+Business Media, LLC, part of Springer Nature, 2019. DOI: [10.1007/s10514-019-09863-2](https://doi.org/10.1007/s10514-019-09863-2)

### Key Concepts Implemented from the Paper:
- **Agile Maneuver Space**: The planner uses a fixed set of motion primitives (maneuvers) per step to discretize the UAV's control inputs, replicating the maneuver-space approach proposed in the paper.
- **Motion Primitives**: Maneuvers such as *Steady Cruise Straight*, *Standard Left/Right Turn*, *Knife-Edge Sharp Turn*, *Agile Climb/Dive*, and *Agile Climbing Turns* are directly modeled from the agile maneuver categories defined in the paper.
- **RRT-based Path Planning**: The Rapidly-exploring Random Tree (RRT) algorithm is applied within the maneuver space to find a real-time, collision-free path from start to goal.
- **Yaw & Pitch Kinematic Constraints**: Maximum yaw and pitch change per step are enforced to respect the UAV's physical turning limits, consistent with the fixed-wing UAV dynamics described in the paper.

