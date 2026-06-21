"""
gui_visualizer.py  -  Ground Control Panel UI (Version 2)
=========================================================
Desktop front-end of the hybrid simulation pipeline.

Responsibilities
----------------
1. Control panel (Tkinter): goal X/Y/Z input fields + a "RUN PLANNER" button.
2. Background UDP listener thread bound to 127.0.0.1:5005 (always running).
3. On "RUN PLANNER": launch the compiled C++ engine as a subprocess, passing
   the goal coordinates as command-line arguments.
4. When the engine streams the solved path back over UDP, parse it and render
   the 3D airspace (grid, obstacles, start, goal, resolved path) inside an
   embedded matplotlib canvas.

Pipeline:
    [GUI]  --launch subprocess(goal)-->  [rrt_engine]
    [rrt_engine]  --UDP path string :5005-->  [GUI listener]  --render-->

Dependencies:
    - Python 3.8+
    - matplotlib            (pip install matplotlib)
    - Tkinter               (ships with most Python builds; on Debian/Ubuntu:
                             sudo apt-get install python3-tk)
    - The compiled engine `rrt_engine` (or `rrt_engine.exe` on Windows) sitting
      next to this script. Build it with:
          g++ -std=c++17 -O2 main.cpp -o rrt_engine            (Linux/macOS)
          g++ -std=c++17 -O2 main.cpp -o rrt_engine.exe -lws2_32 (Windows)

Run:  python gui_visualizer.py
"""

import os
import queue
import socket
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")  # Embed matplotlib inside the Tk window.
from matplotlib.backends.backend_tkagg import (  # noqa: E402
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

# --------------------------------------------------------------------------- #
#  Shared configuration (must match the C++ engine exactly)
# --------------------------------------------------------------------------- #
X_MAX, Y_MAX, Z_MAX = 100.0, 100.0, 30.0
INFLATION = 1.5
UDP_HOST, UDP_PORT = "127.0.0.1", 5005

OBSTACLES = [
    (20.0, 20.0,  0.0, 40.0, 40.0, 25.0),
    (60.0, 50.0,  0.0, 75.0, 70.0, 30.0),
    (45.0, 10.0,  0.0, 55.0, 60.0, 15.0),
]

START = (5.0, 5.0, 5.0)


def engine_path():
    """Locate the compiled engine binary next to this script."""
    here = os.path.dirname(os.path.abspath(__file__))
    name = "rrt_engine.exe" if os.name == "nt" else "rrt_engine"
    return os.path.join(here, name)


# --------------------------------------------------------------------------- #
#  UDP listener (runs in a background thread)
# --------------------------------------------------------------------------- #
class UdpListener(threading.Thread):
    """Listen on UDP :5005 and push received payloads onto a thread-safe queue."""

    def __init__(self, out_queue):
        super().__init__(daemon=True)
        self.out_queue = out_queue
        self._stop = threading.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((UDP_HOST, UDP_PORT))
        self.sock.settimeout(0.5)  # periodic wake-up so we can stop cleanly

    def run(self):
        while not self._stop.is_set():
            try:
                data, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self.out_queue.put(data.decode("utf-8", errors="replace"))

    def stop(self):
        self._stop.set()
        try:
            self.sock.close()
        except OSError:
            pass


# --------------------------------------------------------------------------- #
#  Parsing
# --------------------------------------------------------------------------- #
def parse_path(payload):
    """
    Convert "x1,y1,z1;x2,y2,z2;..." into a list of (x, y, z) tuples.
    Returns None for an empty payload or the "FAIL" sentinel.
    """
    payload = payload.strip()
    if not payload or payload == "FAIL":
        return None
    pts = []
    for chunk in payload.split(";"):
        parts = chunk.split(",")
        if len(parts) != 3:
            continue
        pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return pts or None


# --------------------------------------------------------------------------- #
#  Main application
# --------------------------------------------------------------------------- #
class GroundControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UAV Ground Control - 3D RRT Planner")
        self.root.geometry("1000x720")

        self.msg_queue = queue.Queue()
        self.listener = UdpListener(self.msg_queue)
        self.listener.start()

        self._build_controls()
        self._build_canvas()
        self._draw_scene(path=None)  # initial empty airspace

        # Poll the queue on the main thread (~10 Hz). All Tk/matplotlib calls
        # MUST happen here, never inside the listener thread.
        self.root.after(100, self._poll_queue)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ----------------------------- UI layout ------------------------------- #
    def _build_controls(self):
        bar = ttk.Frame(self.root, padding=8)
        bar.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(bar, text="Goal X:").pack(side=tk.LEFT)
        self.gx = ttk.Entry(bar, width=7); self.gx.insert(0, "90")
        self.gx.pack(side=tk.LEFT, padx=(2, 10))

        ttk.Label(bar, text="Goal Y:").pack(side=tk.LEFT)
        self.gy = ttk.Entry(bar, width=7); self.gy.insert(0, "90")
        self.gy.pack(side=tk.LEFT, padx=(2, 10))

        ttk.Label(bar, text="Goal Z:").pack(side=tk.LEFT)
        self.gz = ttk.Entry(bar, width=7); self.gz.insert(0, "20")
        self.gz.pack(side=tk.LEFT, padx=(2, 10))

        self.run_btn = ttk.Button(bar, text="RUN PLANNER",
                                  command=self.on_run_planner)
        self.run_btn.pack(side=tk.LEFT, padx=10)

        self.status = ttk.Label(bar, text="Ready.", foreground="gray")
        self.status.pack(side=tk.LEFT, padx=10)

    def _build_canvas(self):
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")

        frame = ttk.Frame(self.root)
        frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, frame)  # pan/zoom/rotate toolbar

    # --------------------------- Event handlers ---------------------------- #
    def on_run_planner(self):
        """Validate goal input, then launch the C++ engine as a subprocess."""
        try:
            gx, gy, gz = float(self.gx.get()), float(self.gy.get()), float(self.gz.get())
        except ValueError:
            messagebox.showerror("Invalid input",
                                 "Goal X, Y, Z must all be numbers.")
            return

        if not (0 <= gx <= X_MAX and 0 <= gy <= Y_MAX and 0 <= gz <= Z_MAX):
            messagebox.showwarning(
                "Out of bounds",
                f"Goal must lie within 0..{X_MAX}, 0..{Y_MAX}, 0..{Z_MAX}.")
            return

        exe = engine_path()
        if not os.path.exists(exe):
            messagebox.showerror(
                "Engine not found",
                f"Could not find the compiled engine at:\n{exe}\n\n"
                "Build it first:\n"
                "  g++ -std=c++17 -O2 main.cpp -o rrt_engine")
            return

        self.status.config(text="Running planner...", foreground="darkorange")
        self.run_btn.config(state=tk.DISABLED)

        # Launch the engine in its own thread so the UI stays responsive while
        # it computes; the solved path returns asynchronously over UDP.
        threading.Thread(
            target=self._launch_engine,
            args=(exe, gx, gy, gz),
            daemon=True,
        ).start()

    def _launch_engine(self, exe, gx, gy, gz):
        try:
            proc = subprocess.run(
                [exe, str(gx), str(gy), str(gz)],
                capture_output=True, text=True, timeout=30)
            if proc.returncode != 0:
                self.msg_queue.put(f"__ERR__Engine exited with code "
                                   f"{proc.returncode}: {proc.stderr.strip()}")
        except subprocess.TimeoutExpired:
            self.msg_queue.put("__ERR__Engine timed out.")
        except OSError as exc:
            self.msg_queue.put(f"__ERR__Failed to launch engine: {exc}")

    def _poll_queue(self):
        """Drain the listener queue on the main thread and react to messages."""
        try:
            while True:
                payload = self.msg_queue.get_nowait()
                if payload.startswith("__ERR__"):
                    self._finish(error=payload[len("__ERR__"):])
                else:
                    self._handle_payload(payload)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)

    def _handle_payload(self, payload):
        path = parse_path(payload)
        if path is None:
            self._finish(error="Engine reported no feasible path (FAIL).")
            return
        self._draw_scene(path=path)
        self._finish(ok=f"Path received: {len(path)} waypoints.")

    def _finish(self, ok=None, error=None):
        self.run_btn.config(state=tk.NORMAL)
        if error:
            self.status.config(text=error, foreground="red")
        elif ok:
            self.status.config(text=ok, foreground="green")

    # ----------------------------- Rendering ------------------------------- #
    @staticmethod
    def _draw_box(ax, box, color="dimgray", alpha=0.75, buffer=0.0):
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
            [v[0], v[1], v[2], v[3]], [v[4], v[5], v[6], v[7]],
            [v[0], v[1], v[5], v[4]], [v[2], v[3], v[7], v[6]],
            [v[1], v[2], v[6], v[5]], [v[0], v[3], v[7], v[4]],
        ]
        ax.add_collection3d(Poly3DCollection(
            faces, facecolors=color, edgecolors="k", linewidths=0.4, alpha=alpha))

    def _draw_scene(self, path):
        self.ax.clear()

        # Obstacles + faint inflated clearance shell.
        for box in OBSTACLES:
            self._draw_box(self.ax, box, color="dimgray", alpha=0.75)
            self._draw_box(self.ax, box, color="orange", alpha=0.08,
                           buffer=INFLATION)

        # Resolved path (thick red line + small node markers).
        if path:
            px, py, pz = zip(*path)
            self.ax.plot(px, py, pz, color="red", linewidth=3.0,
                         marker="o", markersize=3, zorder=5,
                         label="Resolved path")

        # Start (green triangle) and goal (red star).
        self.ax.scatter(*START, color="green", marker="^", s=160,
                        depthshade=False, zorder=6, label="Start")
        if path:
            self.ax.scatter(*path[-1], color="red", marker="*", s=260,
                            depthshade=False, zorder=6, label="Goal")

        self.ax.set_xlim(0, X_MAX)
        self.ax.set_ylim(0, Y_MAX)
        self.ax.set_zlim(0, Z_MAX)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self.ax.set_title("3D RRT UAV Motion Planner")
        self.ax.view_init(elev=28, azim=-58)
        if path:
            self.ax.legend(loc="upper left")
        self.canvas.draw()

    # ------------------------------ Shutdown ------------------------------- #
    def _on_close(self):
        self.listener.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    GroundControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
