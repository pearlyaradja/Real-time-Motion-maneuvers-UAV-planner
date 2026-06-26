import os
import socket
import subprocess
import sys
import matplotlib
matplotlib.use("Agg")  # Non-interactive backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Shared configuration
X_MAX, Y_MAX, Z_MAX = 100.0, 100.0, 30.0
INFLATION = 1.5
UDP_HOST, UDP_PORT = "127.0.0.1", 5005
START = (5.0, 5.0, 5.0)
OBSTACLES = [
    (20.0,  0.0,  0.0, 25.0, 96.0, 30.0),
    (32.0, 40.0,  0.0, 38.0, 60.0, 30.0),
    (45.0,  4.0,  0.0, 50.0, 100.0, 30.0),
    (57.0, 40.0,  0.0, 63.0, 60.0, 30.0),
    (70.0,  0.0,  0.0, 75.0, 96.0, 30.0),
    (82.0,  4.0,  0.0, 87.0, 100.0, 30.0),
]

def parse_path(payload):
    payload = payload.strip()
    if not payload or payload == "FAIL": return None
    pts = []
    for chunk in payload.split(";"):
        if not chunk.strip(): continue
        parts = chunk.split(",")
        pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return pts

def draw_box(ax, box, color="dimgray", alpha=0.75, buffer=0.0):
    x_min, y_min, z_min, x_max, y_max, z_max = box
    x_min -= buffer; y_min -= buffer; z_min -= buffer
    x_max += buffer; y_max += buffer; z_max += buffer
    v = [[x_min, y_min, z_min], [x_max, y_min, z_min], [x_max, y_max, z_min], [x_min, y_max, z_min],
         [x_min, y_min, z_max], [x_max, y_min, z_max], [x_max, y_max, z_max], [x_min, y_max, z_max]]
    faces = [[v[0],v[1],v[2],v[3]], [v[4],v[5],v[6],v[7]], [v[0],v[1],v[5],v[4]],
             [v[2],v[3],v[7],v[6]], [v[1],v[2],v[6],v[5]], [v[0],v[3],v[7],v[4]]]
    ax.add_collection3d(Poly3DCollection(faces, facecolors=color, edgecolors="k", linewidths=0.4, alpha=alpha))

def main():
    # 1. Setup UDP Listener
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, UDP_PORT))
    sock.settimeout(10.0)

    # 2. Launch Engine
    # Default goal
    gx, gy, gz = 90.0, 90.0, 20.0
    
    # Override with CLI arguments if provided
    if len(sys.argv) >= 4:
        try:
            gx, gy, gz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
        except ValueError:
            print("Warning: Invalid arguments. Using default goal (90, 90, 20)")

    exe = "./rrt_engine"
    if not os.path.exists(exe):
        print("Error: rrt_engine not found. Compile it first.")
        return

    print(f"Launching engine for goal ({gx}, {gy}, {gz})...")
    subprocess.Popen([exe, str(gx), str(gy), str(gz)])

    # 3. Wait for data
    try:
        data, _ = sock.recvfrom(65535)
        payload = data.decode("utf-8")
        path = parse_path(payload)
    except socket.timeout:
        print("Timed out waiting for path.")
        return
    finally:
        sock.close()

    if not path:
        print("No path found.")
        return

    # 4. Render and Save
    print(f"Path found with {len(path)} points. Generating image...")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    for box in OBSTACLES:
        draw_box(ax, box, color="dimgray", alpha=0.5)
        draw_box(ax, box, color="orange", alpha=0.1, buffer=INFLATION)

    px, py, pz = zip(*path)
    ax.plot(px, py, pz, color="red", linewidth=2, marker="o", markersize=2, label="RRT Path")
    ax.scatter(*START, color="green", marker="^", s=100, label="Start")
    ax.scatter(*path[-1], color="red", marker="*", s=150, label="Goal")

    ax.set_xlim(0, X_MAX); ax.set_ylim(0, Y_MAX); ax.set_zlim(0, Z_MAX)
    ax.set_title("UAV Path - Cloud Shell Headless Export")
    ax.legend()
    
    output_file = "path_result.png"
    plt.savefig(output_file)
    print(f"Success! Image saved to: {os.path.abspath(output_file)}")

if __name__ == "__main__":
    main()