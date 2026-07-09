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
A modern, interactive 3D web-based visualization that requires **no installation** and runs directly in the browser. Each motion primitive is produced by numerically integrating a simplified **6-DOF rigid-body flight dynamics model** (`dynamics.js`), so the planner only accepts physically realizable maneuvers — any primitive that stalls the aircraft is rejected before it enters the RRT tree.

#### Features:
- Rendered in real-time WebGL using **Three.js** and **OrbitControls**.
- Supports multiple UAVs path planning with inter-UAV collision avoidance.
- **6-DOF dynamics engine** (`dynamics.js`): linear aerodynamic stability derivatives, RK4 integration, stall detection, and trim solving for level flight and coordinated turns.
- **Analytic Steer** (`steerAnalytic`): closed-form circular-arc propagation aimed directly at each RRT sample (O(1) per candidate), mirroring the paper's `Steer()` function.
- **k-Nearest retry**: tries up to 5 nearest nodes before discarding a sample, reducing wasted iterations near obstacles.
- **Trim Airspeed slider**: adjusts the aircraft cruise speed (4–12 m/s); all trim conditions and primitive durations update automatically.
- **Stall Rejects counter**: live readout of how many candidates were rejected by the 6-DOF feasibility check, proving the dynamics filter is active.
- Animates tree-growth step-by-step or instantly solves path with live `SEARCHING... X%` progress.
- **Primitive label column** in the waypoint table shows which maneuver produced each node.
- Interactive 3D controls (left-click to rotate, right-click to pan, scroll to zoom).

#### Web Planner File Structure:
| File | Role |
|---|---|
| `index.html` | UI layout, sliders, stats dashboard, waypoint table |
| `style.css` | Dark-mode styling, stats grid, responsive layout |
| `dynamics.js` | 6-DOF rigid-body model, RK4, trim solvers, stall detection |
| `planner.js` | RRT core: Node (12-state), `steerAnalytic`, `generatePrimitives`, collision check |
| `app.js` | Three.js scene, `runPlannerBatched`, flight animation, UI wiring |
| `three.min.js` | Three.js r128 (bundled, no CDN needed for offline use) |
| `OrbitControls.js` | Three.js orbit camera controller |

#### How to Run:
* **Option A: Using Live Server Extension (Recommended)**
  1. Open this project in **VS Code**.
  2. Install the **Live Server** extension (by Ritwick Dey).
  3. Open `web_planner/index.html`, right-click in the editor, and select **"Open with Live Server"**.
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
  2. Double-click `index.html` to open it in any browser (no internet connection required — Three.js is bundled locally).

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
- **Agile Maneuver Space**: The planner uses a fixed set of motion primitives per step to discretize the UAV's control inputs, replicating the maneuver-space approach proposed in the paper. In the Web Planner, each primitive is produced by integrating the 6-DOF equations of motion under a fixed control input rather than a geometric yaw/pitch offset.
- **Motion Primitives**: Maneuvers such as *Steady Cruise Straight*, *Standard Left/Right Turn*, *Knife-Edge Sharp Turn*, *Agile Climb/Dive*, and *Agile Climbing Turns* are directly modeled from the agile maneuver categories defined in the paper. Trim conditions (control inputs for steady flight) are solved analytically for each primitive.
- **Analytic Steer (Sect. 6.3 / Eq. 3)**: The Web Planner implements the paper's closed-form `Steer()` function using circular-arc geometry. Given any RRT sample, it solves the required yaw rate and climb rate analytically via `solveTurnTrim()` and propagates the resulting arc in O(1) — no simulation loop per candidate.
- **Dynamic Feasibility / Stall Rejection**: Any candidate primitive that exceeds the aircraft's trim-able envelope (too tight a turn, too steep a climb, or stall angle-of-attack exceeded) is rejected before being added to the tree. The UI shows a live **Stall Rejects** count as evidence that this filter is active.
- **RRT-based Path Planning**: The Rapidly-exploring Random Tree (RRT) algorithm is applied within the maneuver space to find a real-time, collision-free path from start to goal.
- **Yaw & Pitch Kinematic Constraints**: For the Web Planner, constraints emerge naturally from the 6-DOF trim model (bank angle, control surface limits, thrust limit). For the Python version, maximum yaw and pitch change per step are enforced explicitly.
- **k-Nearest Retry**: The Web Planner retries up to 5 nearest nodes before discarding a random sample, consistent with the "extend tree" robustness described in Sect. 6.2 / Algorithm 2 of the paper.
