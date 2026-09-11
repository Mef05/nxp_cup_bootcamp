"""
simulate.py - NXP Cup race car simulation with matplotlib visualisation.

Controls:
    SPACE   - pause / resume
    R       - reset car to start position
    +       - increase simulation speed multiplier
    -       - decrease simulation speed multiplier
    Q / Esc - quit

Layout (3 subplots):
    Left            : Bird's-eye view of track + car + camera FOV cone
    Top-right       : Simulated Pixy2 camera view (79x51 pixel grid + vectors)
    Bottom-right    : Time-series: steer, speed, CTE, proximity (last 200 frames)

Console output (every PRINT_EVERY_N frames):
    Same format as serial log: "L+R cbot:X cte:X hdg:X prox:X err:X"
    Followed by:               "STR:X SPD_L:X SPD_R:X"

Run:
    python simulate.py
"""

from __future__ import annotations

import math
import sys
import time
from collections import deque
from typing import Deque, List, Tuple

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrow

from bicycle_model import BicycleModel
from camera import Pixy2Camera
from config import (
    CAMERA_ANGLE_DEG,
    CAMERA_FOV_H_DEG,
    CAMERA_HEIGHT,
    IMAGE_CENTER_X,
    PIXY_H,
    PIXY_W,
    PRINT_EVERY_N,
    SPEED_LEFT,
    SPEED_RIGHT,
    TRACK_WIDTH,
)
from controller import NXPController
from track import (
    find_closest_point,
    get_track_boundaries,
    get_track_centerline,
)

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------

DT: float = 1.0 / 60.0      # 60 Hz frame rate (matching camera frame rate)
HISTORY_LEN: int = 200       # frames kept in time-series plots
SPEED_STEPS: List[float] = [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]

# Initial car pose - placed at first centreline point facing along track
_centreline = get_track_centerline()
_x0, _y0 = _centreline[0]
# Approximate initial heading from first two points
if len(_centreline) > 1:
    _dx = _centreline[1][0] - _centreline[0][0]
    _dy = _centreline[1][1] - _centreline[0][1]
    _theta0 = math.atan2(_dy, _dx)
else:
    _theta0 = 0.0

# ---------------------------------------------------------------------------
# SimulationRunner
# ---------------------------------------------------------------------------

class SimulationRunner:
    """Runs the full simulation loop and maintains visualisation state."""

    def __init__(self) -> None:
        # Build track geometry once
        self.centreline = get_track_centerline()
        self.left_boundary, self.right_boundary = get_track_boundaries()

        # Sub-systems
        self.car = BicycleModel(_x0, _y0, _theta0)
        self.camera = Pixy2Camera()
        self.controller = NXPController()

        # Simulation state
        self.paused: bool = False
        self.speed_idx: int = 2  # default 1x
        self.frame: int = 0

        # Time-series history
        self.hist_steer: Deque[float] = deque(maxlen=HISTORY_LEN)
        self.hist_speed: Deque[float] = deque(maxlen=HISTORY_LEN)
        self.hist_cte: Deque[float] = deque(maxlen=HISTORY_LEN)
        self.hist_prox: Deque[float] = deque(maxlen=HISTORY_LEN)

        # Car trajectory for bird's-eye path trace
        self.path_x: Deque[float] = deque(maxlen=500)
        self.path_y: Deque[float] = deque(maxlen=500)

        # Latest vectors for camera view
        self.last_vectors: List[Tuple[int, int, int, int]] = []
        self.last_debug: dict = {}

    def reset(self) -> None:
        self.car.reset(_x0, _y0, _theta0)
        self.controller.reset()
        self.frame = 0
        self.hist_steer.clear()
        self.hist_speed.clear()
        self.hist_cte.clear()
        self.hist_prox.clear()
        self.path_x.clear()
        self.path_y.clear()
        self.last_vectors = []
        self.last_debug = {}

    def step(self) -> None:
        """Advance one simulation frame."""
        # Get camera vectors
        vectors = self.camera.get_vectors(
            self.car.x, self.car.y, self.car.theta,
            self.left_boundary, self.right_boundary,
        )
        self.last_vectors = vectors

        # Run controller (mirrors main.c loop body)
        steer_out, speed_L, speed_R, debug = self.controller.update(vectors)
        self.last_debug = debug

        # Average speed for bicycle model (use mean of L+R)
        avg_speed = (speed_L + speed_R) / 2.0

        # Advance bicycle model
        self.car.step(steer_out, avg_speed, DT)

        # Track path
        self.path_x.append(self.car.x)
        self.path_y.append(self.car.y)

        # Find CTE from world track
        _, _, cte_world = find_closest_point(self.car.x, self.car.y)

        # Append to history
        self.hist_steer.append(steer_out)
        self.hist_speed.append(avg_speed)
        self.hist_cte.append(cte_world)
        self.hist_prox.append(debug.get("proximity", 0.0))

        # Console log (every PRINT_EVERY_N frames, mirrors main.c format)
        self.frame += 1
        if self.frame % PRINT_EVERY_N == 0:
            have_l = debug.get("have_left", False)
            have_r = debug.get("have_right", False)
            tag = "L+R" if (have_l and have_r) else ("L  " if have_l else "  R")
            print(
                f"{tag} "
                f"cbot:{int(debug.get('center_bot', IMAGE_CENTER_X))} "
                f"cte:{int(debug.get('cte', 0))} "
                f"hdg:{int(debug.get('heading', 0))} "
                f"prox:{int(debug.get('proximity', 0) * 100)} "
                f"err:{int(debug.get('error', 0))}"
            )
            print(
                f"STR:{int(steer_out)} "
                f"SPD_L:{speed_L} "
                f"SPD_R:{speed_R}"
            )


# ---------------------------------------------------------------------------
# Matplotlib setup and animation
# ---------------------------------------------------------------------------

def _build_figure():
    """Build the 3-panel figure layout."""
    fig = plt.figure(figsize=(14, 8), facecolor="#1a1a2e")
    # GridSpec: left large panel + right two stacked panels
    gs = fig.add_gridspec(2, 2, width_ratios=[1.6, 1], hspace=0.35, wspace=0.3,
                          left=0.06, right=0.97, top=0.93, bottom=0.08)

    ax_bird = fig.add_subplot(gs[:, 0])   # bird's-eye (spans both rows)
    ax_cam  = fig.add_subplot(gs[0, 1])   # camera view
    ax_ts   = fig.add_subplot(gs[1, 1])   # time series

    for ax in (ax_bird, ax_cam, ax_ts):
        ax.set_facecolor("#0d0d1a")
        for spine in ax.spines.values():
            spine.set_color("#444466")
        ax.tick_params(colors="#8888aa", labelsize=7)

    return fig, ax_bird, ax_cam, ax_ts


def run_simulation(non_interactive: bool = False, max_frames: int = 0) -> None:
    """
    Main entry point.

    Args:
        non_interactive : if True, run headlessly for testing (no GUI window).
        max_frames      : stop after this many frames (0 = run forever).
    """
    sim = SimulationRunner()

    if non_interactive:
        # Headless test mode - just run a few frames
        print("=== Non-interactive test mode ===")
        for _ in range(max_frames or 120):
            sim.step()
        print(f"Ran {sim.frame} frames without errors.")
        print(f"Final car position: ({sim.car.x:.3f}, {sim.car.y:.3f}), "
              f"heading: {math.degrees(sim.car.theta):.1f} deg")
        return

    # -----------------------------------------------------------------------
    # Interactive matplotlib animation
    # -----------------------------------------------------------------------
    fig, ax_bird, ax_cam, ax_ts = _build_figure()
    fig.suptitle("NXP Cup Race Car Simulation", color="#ccccee", fontsize=12, y=0.98)

    # -- Bird's-eye static track elements --
    cx = [p[0] for p in sim.centreline]
    cy = [p[1] for p in sim.centreline]
    lx = [p[0] for p in sim.left_boundary]
    ly = [p[1] for p in sim.left_boundary]
    rx = [p[0] for p in sim.right_boundary]
    ry = [p[1] for p in sim.right_boundary]

    ax_bird.plot(cx, cy, "--", color="#333355", lw=0.8, label="centreline")
    ax_bird.plot(lx, ly, "-", color="#2255aa", lw=1.5, label="left boundary")
    ax_bird.plot(rx, ry, "-", color="#aa2222", lw=1.5, label="right boundary")
    ax_bird.set_aspect("equal")
    ax_bird.set_title("Bird's-eye view", color="#ccccee", fontsize=9)
    ax_bird.set_xlabel("X (m)", color="#8888aa", fontsize=7)
    ax_bird.set_ylabel("Y (m)", color="#8888aa", fontsize=7)

    # Dynamic bird's-eye elements
    (path_line,) = ax_bird.plot([], [], "-", color="#44ff88", lw=0.8, alpha=0.6)
    car_arrow = ax_bird.annotate(
        "", xy=(0, 0), xytext=(-0.1, 0),
        arrowprops=dict(arrowstyle="->", color="#ffff00", lw=2),
    )
    # FOV cone (two lines from car forward)
    (fov_l,) = ax_bird.plot([], [], "-", color="#ffaa00", lw=0.8, alpha=0.5)
    (fov_r,) = ax_bird.plot([], [], "-", color="#ffaa00", lw=0.8, alpha=0.5)
    speed_text = ax_bird.text(
        0.02, 0.97, "", transform=ax_bird.transAxes,
        color="#ffff88", fontsize=7, va="top", family="monospace",
    )

    # -- Camera view --
    ax_cam.set_xlim(-0.5, PIXY_W - 0.5)
    ax_cam.set_ylim(PIXY_H - 0.5, -0.5)   # Y=0 at top
    ax_cam.set_title("Pixy2 camera view (79×51 px)", color="#ccccee", fontsize=9)
    ax_cam.set_xlabel("X pixel", color="#8888aa", fontsize=7)
    ax_cam.set_ylabel("Y pixel (0=far, 50=close)", color="#8888aa", fontsize=7)
    # Centre vertical line
    ax_cam.axvline(IMAGE_CENTER_X, color="#555577", lw=0.8, ls="--")
    # MIN_BOT_Y horizontal guide
    from config import MIN_BOT_Y as _MINBY
    ax_cam.axhline(_MINBY, color="#443333", lw=0.8, ls=":")
    cam_vectors_lines: list = []  # will hold Line2D objects

    # -- Time-series --
    ax_ts.set_title("Time series (last 200 frames)", color="#ccccee", fontsize=9)
    ax_ts.set_xlabel("frame", color="#8888aa", fontsize=7)
    ax_ts.set_xlim(0, HISTORY_LEN)
    ax_ts.set_ylim(-110, 110)
    ax_ts.axhline(0, color="#333355", lw=0.5)
    (ts_steer,) = ax_ts.plot([], [], "-", color="#ffff44", lw=1.2, label="steer")
    (ts_speed,) = ax_ts.plot([], [], "-", color="#44ff88", lw=1.2, label="speed")
    (ts_cte,)   = ax_ts.plot([], [], "-", color="#ff8844", lw=1.0, label="CTE×20")
    (ts_prox,)  = ax_ts.plot([], [], "-", color="#aa44ff", lw=1.0, label="prox×100")
    ax_ts.legend(loc="upper right", fontsize=6, facecolor="#111122", labelcolor="#ccccee",
                 framealpha=0.8)

    # UI state
    ui = {"paused": False, "speed_idx": 2, "quit": False}

    def on_key(event):
        if event.key == " ":
            ui["paused"] = not ui["paused"]
        elif event.key == "r":
            sim.reset()
        elif event.key in ("+", "="):
            ui["speed_idx"] = min(len(SPEED_STEPS) - 1, ui["speed_idx"] + 1)
        elif event.key == "-":
            ui["speed_idx"] = max(0, ui["speed_idx"] - 1)
        elif event.key in ("q", "escape"):
            ui["quit"] = True
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def _make_fov_line(car_x, car_y, car_theta, angle_offset, length=2.5):
        """Return two-point (x,y) line for one edge of the FOV cone."""
        angle = car_theta + angle_offset
        return (
            [car_x, car_x + length * math.cos(angle)],
            [car_y, car_y + length * math.sin(angle)],
        )

    fov_half_h = math.radians(CAMERA_FOV_H_DEG / 2.0)

    def animate(frame_num):
        if ui.get("quit"):
            return

        if not ui["paused"]:
            mult = SPEED_STEPS[ui["speed_idx"]]
            steps = max(1, int(mult))
            for _ in range(steps):
                sim.step()
                if max_frames and sim.frame >= max_frames:
                    plt.close(fig)
                    return

        cx_car, cy_car = sim.car.x, sim.car.y
        th = sim.car.theta

        # -- Bird's-eye update --
        path_line.set_data(list(sim.path_x), list(sim.path_y))
        # Car arrow: tail at car pos, tip 0.2m ahead
        arrow_len = 0.2
        car_arrow.xy = (cx_car + arrow_len * math.cos(th), cy_car + arrow_len * math.sin(th))
        car_arrow.xytext = (cx_car, cy_car)

        fl_x, fl_y = _make_fov_line(cx_car, cy_car, th, fov_half_h)
        fr_x, fr_y = _make_fov_line(cx_car, cy_car, th, -fov_half_h)
        fov_l.set_data(fl_x, fl_y)
        fov_r.set_data(fr_x, fr_y)

        mult_val = SPEED_STEPS[ui["speed_idx"]]
        state_str = "PAUSED" if ui["paused"] else f"{mult_val:.2f}x"
        spd_L = sim.last_debug.get("speed_L", 0)
        spd_R = sim.last_debug.get("speed_R", 0)
        speed_text.set_text(
            f"[{state_str}]  STR:{sim.controller.current_steer:+.1f}  "
            f"SPD_L:{spd_L}  SPD_R:{spd_R}\n"
            f"X:{cx_car:.2f}m  Y:{cy_car:.2f}m  "
            f"frames_lost:{sim.controller.frames_lost}"
        )

        # Auto-scroll the bird's-eye view to follow car
        margin = 1.5
        ax_bird.set_xlim(cx_car - margin, cx_car + margin)
        ax_bird.set_ylim(cy_car - margin, cy_car + margin)

        # -- Camera view update --
        for ln in cam_vectors_lines:
            ln.remove()
        cam_vectors_lines.clear()

        colors = ["#4488ff", "#ff4444", "#44ff44", "#ffff44"]
        for idx, vec in enumerate(sim.last_vectors):
            x0, y0, x1, y1 = vec
            col = colors[idx % len(colors)]
            (ln,) = ax_cam.plot([x0, x1], [y0, y1], "-o", color=col, lw=2, ms=3)
            cam_vectors_lines.append(ln)

        # -- Time-series update --
        n = len(sim.hist_steer)
        xs = list(range(n))
        ts_steer.set_data(xs, list(sim.hist_steer))
        ts_speed.set_data(xs, list(sim.hist_speed))
        ts_cte.set_data(xs, [v * 20.0 for v in sim.hist_cte])
        ts_prox.set_data(xs, [v * 100.0 for v in sim.hist_prox])

        return (path_line, car_arrow, fov_l, fov_r, speed_text,
                ts_steer, ts_speed, ts_cte, ts_prox)

    ani = FuncAnimation(fig, animate, interval=16, blit=False, cache_frame_data=False)

    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Allow non-interactive test flag: python simulate.py --test [N_frames]
    if "--test" in sys.argv:
        idx = sys.argv.index("--test")
        n = int(sys.argv[idx + 1]) if idx + 1 < len(sys.argv) else 180
        run_simulation(non_interactive=True, max_frames=n)
    else:
        run_simulation()
