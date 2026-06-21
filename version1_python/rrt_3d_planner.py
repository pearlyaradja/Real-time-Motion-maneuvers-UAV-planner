"""
3D RRT Motion Planner for a UAV - Pure Python Architecture (Version 1)
=====================================================================
A self-contained Rapidly-exploring Random Tree (RRT) planner that finds a
collision-free 3D flight trajectory for an unmanned aerial vehicle inside a
bounded airspace populated with cuboid obstacles.

Features
--------
- Class-based Node structure with parent linkage and cumulative cost.
- Goal-biased uniform sampling (10% bias toward the goal node).
- Modular `steer` local planner (straight segments now, swappable for a
  motion-primitives look-up table in the future).
- Geometric axis-aligned bounding-box collision checking with a 1.5 m
  obstacle inflation buffer for safe aircraft clearance.
- Cumulative cost tracking on every validated node.
- Backward path extraction via parent references once the goal is reached.
- Full matplotlib 3D visualisation: tree branches, obstacle blocks, start
  point, goal point, and the highlighted final path.

Run:  python rrt_3d_planner.py
"""

import math
import random

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --------------------------------------------------------------------------- #
#  Global airspace configuration
# --------------------------------------------------------------------------- #
X_MAX = 100.0            # Airspace width   (metres)
Y_MAX = 100.0            # Airspace depth   (metres)
Z_MAX = 30.0             # Airspace ceiling (metres)

STEP_SIZE = 5.0          # Maximum expansion distance per iteration (metres)
GOAL_SAMPLE_RATE = 0.10  # Probability of sampling the goal directly (10%)
GOAL_THRESHOLD = 5.0     # Distance at which the goal counts as reached (metres)
MAX_ITER = 200           # Iteration budget
INFLATION = 1.5          # Obstacle inflation buffer for clearance (metres)

# Obstacle list. Each cuboid is (x_min, y_min, z_min, x_max, y_max, z_max).
OBSTACLES = [
    (20.0, 20.0,  0.0, 40.0, 40.0, 25.0),
    (60.0, 50.0,  0.0, 75.0, 70.0, 30.0),
    (45.0, 10.0,  0.0, 55.0, 60.0, 15.0),
]


# --------------------------------------------------------------------------- #
#  Node structure
# --------------------------------------------------------------------------- #
class Node:
    """A single vertex in the RRT search tree."""

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None   # Parent Node reference (None for the root/start)
        self.cost = 0.0      # Cumulative path cost from the start node


# --------------------------------------------------------------------------- #
#  Core RRT functions
# --------------------------------------------------------------------------- #
def hitung_jarak(node_a, node_b):
    """Euclidean distance between two nodes (hitung jarak = 'compute distance')."""
    return math.sqrt(
        (node_a.x - node_b.x) ** 2
        + (node_a.y - node_b.y) ** 2
        + (node_a.z - node_b.z) ** 2
    )


def get_random_node(goal):
    """
    Draw a sample node.

    With GOAL_SAMPLE_RATE probability the goal itself is returned (goal
    biasing). Otherwise a point is drawn uniformly across the airspace.
    """
    if random.random() < GOAL_SAMPLE_RATE:
        return Node(goal.x, goal.y, goal.z)
    return Node(
        random.uniform(0.0, X_MAX),
        random.uniform(0.0, Y_MAX),
        random.uniform(0.0, Z_MAX),
    )


def get_nearest_node_id(node_list, rnd_node):
    """Return the index of the node in `node_list` closest to `rnd_node`."""
    distances = [hitung_jarak(n, rnd_node) for n in node_list]
    return distances.index(min(distances))


def steer(from_node, to_node, step=STEP_SIZE):
    """
    Local planner.

    Produce a new node by moving from `from_node` toward `to_node` by at most
    `step` metres along a straight line, then attach parent and cost.

    NOTE: This function is deliberately isolated. To upgrade the planner to use
    a motion-primitives look-up table, replace ONLY the interpolation block
    below with a primitive lookup that returns the reachable end-state; the rest
    of the RRT pipeline stays untouched.
    """
    dist = hitung_jarak(from_node, to_node)

    if dist <= step:
        # Target is within a single step: snap straight to it.
        new_node = Node(to_node.x, to_node.y, to_node.z)
    else:
        # ---- swappable linear-primitive block --------------------------- #
        ratio = step / dist
        new_x = from_node.x + (to_node.x - from_node.x) * ratio
        new_y = from_node.y + (to_node.y - from_node.y) * ratio
        new_z = from_node.z + (to_node.z - from_node.z) * ratio
        new_node = Node(new_x, new_y, new_z)
        # ----------------------------------------------------------------- #

    new_node.parent = from_node
    new_node.cost = from_node.cost + hitung_jarak(from_node, new_node)
    return new_node


def _point_in_inflated_box(x, y, z, box, buffer=INFLATION):
    """True if (x, y, z) lies inside `box` expanded by `buffer` on every side."""
    x_min, y_min, z_min, x_max, y_max, z_max = box
    return (
        x_min - buffer <= x <= x_max + buffer
        and y_min - buffer <= y <= y_max + buffer
        and z_min - buffer <= z <= z_max + buffer
    )


def check_collision(from_node, to_node, obstacles=OBSTACLES):
    """
    Return True if the straight segment from `from_node` to `to_node` is
    collision-free against all inflated obstacle boxes.

    The segment is densely sampled (~every 0.5 m) so thin obstacles cannot be
    tunnelled through.
    """
    dist = hitung_jarak(from_node, to_node)
    steps = max(2, int(dist / 0.5))
    for i in range(steps + 1):
        t = i / steps
        x = from_node.x + (to_node.x - from_node.x) * t
        y = from_node.y + (to_node.y - from_node.y) * t
        z = from_node.z + (to_node.z - from_node.z) * t
        for box in obstacles:
            if _point_in_inflated_box(x, y, z, box):
                return False  # Collision detected -> reject segment
    return True               # Segment is clear


def extract_path(goal_node):
    """Trace parent references from the goal back to the start, then reverse."""
    path = []
    node = goal_node
    while node is not None:
        path.append((node.x, node.y, node.z))
        node = node.parent
    path.reverse()
    return path


# --------------------------------------------------------------------------- #
#  Planner driver
# --------------------------------------------------------------------------- #
def plan(start, goal, seed=None):
    """
    Grow the RRT for up to MAX_ITER iterations.

    Returns (node_list, path) where `path` is a list of (x, y, z) tuples, or
    None if no path was found within the iteration budget.
    """
    if seed is not None:
        random.seed(seed)

    node_list = [start]
    path = None

    for i in range(MAX_ITER):
        # 1. Sample (with goal bias).
        rnd = get_random_node(goal)

        # 2. Find the nearest existing node.
        nearest = node_list[get_nearest_node_id(node_list, rnd)]

        # 3. Steer one step toward the sample.
        new_node = steer(nearest, rnd)

        # 4. Reject if the connecting segment hits an inflated obstacle.
        if not check_collision(nearest, new_node):
            continue

        # 5. Accept the validated node.
        node_list.append(new_node)

        # 6. Goal test: connect to the goal if close enough and clear.
        if hitung_jarak(new_node, goal) <= GOAL_THRESHOLD and check_collision(new_node, goal):
            goal.parent = new_node
            goal.cost = new_node.cost + hitung_jarak(new_node, goal)
            node_list.append(goal)
            path = extract_path(goal)
            print(f"[OK]  Goal reached at iteration {i + 1}  |  "
                  f"path cost = {goal.cost:.2f} m  |  waypoints = {len(path)}")
            break

    if path is None:
        print(f"[FAIL] No path found within {MAX_ITER} iterations.")
    return node_list, path


# --------------------------------------------------------------------------- #
#  Visualisation helpers
# --------------------------------------------------------------------------- #
def _draw_box(ax, box, color="0.5", alpha=0.65, buffer=0.0):
    """Render a single cuboid (optionally inflated by `buffer`) as solid faces."""
    x_min, y_min, z_min, x_max, y_max, z_max = box
    x_min -= buffer; y_min -= buffer; z_min -= buffer
    x_max += buffer; y_max += buffer; z_max += buffer

    v = [
        [x_min, y_min, z_min], [x_max, y_min, z_min],
        [x_max, y_max, z_min], [x_min, y_max, z_min],
        [x_min, y_min, z_max], [x_max, y_min, z_max],
        [x_max, y_max, z_max], [x_min, y_max, z_max],
    ]
    faces = [
        [v[0], v[1], v[2], v[3]],  # bottom
        [v[4], v[5], v[6], v[7]],  # top
        [v[0], v[1], v[5], v[4]],  # sides
        [v[2], v[3], v[7], v[6]],
        [v[1], v[2], v[6], v[5]],
        [v[0], v[3], v[7], v[4]],
    ]
    ax.add_collection3d(
        Poly3DCollection(faces, facecolors=color, edgecolors="k",
                         linewidths=0.4, alpha=alpha)
    )


def visualise(node_list, path, start, goal, save_path=None):
    """Render the airspace, tree, obstacles, endpoints and the final path."""
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Obstacles: solid blocks (+ faint inflated shell to show the buffer).
    for box in OBSTACLES:
        _draw_box(ax, box, color="dimgray", alpha=0.75)
        _draw_box(ax, box, color="orange", alpha=0.08, buffer=INFLATION)

    # Tree branches: light blue lines from each node to its parent.
    for node in node_list:
        if node.parent is not None:
            ax.plot(
                [node.x, node.parent.x],
                [node.y, node.parent.y],
                [node.z, node.parent.z],
                color="lightskyblue", linewidth=0.6, zorder=1,
            )

    # Final path: thick red line.
    if path:
        px, py, pz = zip(*path)
        ax.plot(px, py, pz, color="red", linewidth=3.0,
                zorder=5, label="Optimal trajectory")

    # Start (green triangle) and Goal (red star).
    ax.scatter(start.x, start.y, start.z, color="green", marker="^",
               s=160, depthshade=False, zorder=6, label="Start")
    ax.scatter(goal.x, goal.y, goal.z, color="red", marker="*",
               s=260, depthshade=False, zorder=6, label="Goal")

    ax.set_xlim(0, X_MAX)
    ax.set_ylim(0, Y_MAX)
    ax.set_zlim(0, Z_MAX)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("3D RRT UAV Motion Planner")
    ax.legend(loc="upper left")
    ax.view_init(elev=28, azim=-58)

    if save_path:
        plt.savefig(save_path, dpi=130, bbox_inches="tight")
        print(f"[OK]  Figure saved to {save_path}")
    else:
        plt.show()


# --------------------------------------------------------------------------- #
#  Entry point
# --------------------------------------------------------------------------- #
def main():
    start = Node(5.0, 5.0, 5.0)
    goal = Node(90.0, 90.0, 20.0)

    node_list, path = plan(start, goal, seed=7)
    visualise(node_list, path, start, goal)


if __name__ == "__main__":
    main()
