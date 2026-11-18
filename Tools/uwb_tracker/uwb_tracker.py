"""
UWB tag tracker: ranges from the tag (board B) -> Kalman filter -> live 2D map.

The tag (uwb_tag in the Dw3000 library examples) prints one line per range:
    DIST 1 1.29 m raw 1.33
The first number is the range after the tag's compensation and IIR filter
(uwb_tag v3.0+); lines without "raw" (older tags) are raw ranges and get
bias_m subtracted here. This script reads those lines from its USB serial port, runs an extended
Kalman filter (state x, y, vx, vy; constant-velocity model) that is updated
with every single range, and draws the anchors, the range circles, the raw
least-squares fixes and the filtered track.

Usage:
    python uwb_tracker.py --port COM6                 live, from board B
    python uwb_tracker.py --port COM6 --log run.txt   live, and save the raw lines
    python uwb_tracker.py --replay run.txt            play a saved run back
    python uwb_tracker.py --simulate                  synthetic tag, no hardware
    add --raw to show the unfiltered least-squares fix instead of the EKF

Live: press f in the map window to step the tag's round period (25/100/50/20/10/0 ms).

Anchor positions, per-anchor range compensation (scale, bias_m), the tag's IIR
coefficient (iir_alpha) and noise settings: anchors.json.
"""

import argparse
import json
import math
import os
import queue
import re
import threading
import time
from collections import deque

import numpy as np

# Group 3 set: the tag already compensated and filtered the range (uwb_tag v3.0+).
DIST_RE = re.compile(r"DIST\s+(\S+)\s+(-?\d+(?:\.\d+)?)\s*m(\s+raw\b)?")
ANCHORS_RE = re.compile(r"anchors:(.*)")
PERIOD_RE = re.compile(r"round period (\d+) ms")
RATE_RE = re.compile(r"RATE period (\d+) ms\s+round ([\d.]+) ms")
SPREAD_RE = re.compile(r"SPREAD (\S+) n \d+\s+([\d.]+) Hz")


def load_config(path):
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    anchors = {}
    for tag, a in cfg["anchors"].items():
        anchors[tag] = {
            "name": a.get("name", tag),
            "pos": np.array([float(a["x"]), float(a["y"])]),
            "bias": float(a.get("bias_m", 0.0)),
            "scale": float(a.get("scale", 1.0)),
        }
    map_size = cfg.get("map_size_m")
    return (anchors, float(cfg.get("range_sigma_m", 0.12)), float(cfg.get("accel_sigma_mps2", 0.5)),
            float(map_size) if map_size else None, float(cfg.get("iir_alpha", 0.1)))


# ---------------------------------------------------------------------------
# Range sources. Each puts (t, tag, range_m) on a queue; t is seconds.
# ---------------------------------------------------------------------------

def open_serial(port, baud):
    import serial

    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as e:
        raise SystemExit(f"cannot open {port}: {e}\n(close the Arduino Serial Monitor or any other program using it)")
    ser.dtr = True
    return ser


def send_compensation(ser, anchors, iir_alpha):
    """Give the tag its per-anchor range compensation and IIR coefficient (uwb_tag v3.0+)."""
    for tag, a in anchors.items():
        if tag.isdigit():
            ser.write(f"C{tag} {a['scale']:.4f} {a['bias']:.3f}\n".encode())
            time.sleep(0.02)
    ser.write(f"L{iir_alpha:.3f}\n".encode())


def serial_source(ser, out, anchors, log_path, stop, tag_info, iir_alpha):
    # Line-buffered: this thread is a daemon and may be killed without close().
    log = open(log_path, "a", encoding="utf-8", buffering=1) if log_path else None
    t0 = time.monotonic()
    asked = set()
    buf = b""
    try:
        while not stop.is_set():
            buf += ser.read(256)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                t = time.monotonic() - t0
                if log:
                    log.write(f"{t:.3f} {line}\n")
                m = DIST_RE.search(line)
                if m:
                    out.put((t, m.group(1), float(m.group(2)), bool(m.group(3))))
                    continue
                # Rate report of the tag (uwb_tag v2.1+), shown on the map.
                m = PERIOD_RE.search(line)
                if m:
                    tag_info["period"] = int(m.group(1))
                    tag_info["round"] = None
                    tag_info["hz"] = {}
                    print(f"[serial] {line}")
                    continue
                m = RATE_RE.search(line)
                if m:
                    tag_info["period"] = int(m.group(1))
                    tag_info["round"] = float(m.group(2))
                    continue
                m = SPREAD_RE.search(line)
                if m:
                    tag_info.setdefault("hz", {})[m.group(1)] = float(m.group(2))
                    continue
                m = ANCHORS_RE.search(line)
                if m:
                    # Turn on every anchor in anchors.json that the tag skips
                    # (keys 1..9 toggle the anchor with that ID).
                    enabled = set(m.group(1).split())
                    for tag in anchors:
                        if tag not in enabled and len(tag) == 1 and tag.isdigit() and tag not in asked:
                            ser.write(tag.encode())
                            asked.add(tag)
                            time.sleep(0.05)
                    send_compensation(ser, anchors, iir_alpha)
                    print(f"[serial] {line}")
    finally:
        ser.close()
        if log:
            log.close()


def replay_source(path, out, speed, stop):
    t_start = time.monotonic()
    with open(path, encoding="utf-8") as f:
        for row in f:
            if stop.is_set():
                return
            parts = row.split(" ", 1)
            if len(parts) != 2:
                continue
            try:
                t = float(parts[0])
            except ValueError:
                continue
            m = DIST_RE.search(parts[1])
            if not m:
                continue
            item = (t, m.group(1), float(m.group(2)), bool(m.group(3)))
            wait = t / speed - (time.monotonic() - t_start)
            if wait > 0:
                time.sleep(wait)
            out.put(item)


def simulate_source(anchors, sigma, out, stop):
    """Tag walks a figure-eight inside the anchor area; one range per anchor every 0.17 s."""
    rng = np.random.default_rng(1)
    pts = np.array([a["pos"] for a in anchors.values()])
    centre = pts.mean(axis=0)
    span = max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1]), 1.0) * 0.3
    t0 = time.monotonic()
    while not stop.is_set():
        for tag, a in anchors.items():
            t = time.monotonic() - t0
            w = 2 * math.pi * t / 20.0
            p = centre + span * np.array([math.sin(w), math.sin(2 * w) / 2])
            r = np.linalg.norm(p - a["pos"]) + a["bias"] + rng.normal(0, sigma)
            if rng.random() < 0.03:
                r += rng.uniform(0.5, 1.5)  # occasional NLOS-like outlier
            out.put((t, tag, max(r, 0.0), False))
            time.sleep(0.5 / 3)


# ---------------------------------------------------------------------------
# Positioning
# ---------------------------------------------------------------------------

def least_squares_fix(anchor_pos, ranges, guess):
    """Gauss-Newton on sum (|p - a_i| - r_i)^2. Needs 3+ anchors.
    Returns (position, rms range residual) or None."""
    p = np.array(guess, dtype=float)
    for _ in range(15):
        d = p - anchor_pos
        dist = np.linalg.norm(d, axis=1)
        dist[dist < 1e-6] = 1e-6
        J = d / dist[:, None]
        res = dist - ranges
        try:
            step = np.linalg.lstsq(J, res, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None
        p -= step
        if np.linalg.norm(step) < 1e-4:
            break
    res = np.linalg.norm(p - anchor_pos, axis=1) - ranges
    return p, float(np.sqrt(np.mean(res ** 2)))


class RangeEKF:
    """Constant-velocity EKF updated with one range at a time."""

    GATE = 3.0  # reject a range whose innovation is beyond 3 sigma

    def __init__(self, range_sigma, accel_sigma):
        self.r_var = range_sigma ** 2
        self.q = accel_sigma ** 2
        self.x = None
        self.P = None
        self.t = None
        self.rejected_in_row = 0

    def init(self, pos, t):
        self.x = np.array([pos[0], pos[1], 0.0, 0.0])
        self.P = np.diag([0.3 ** 2, 0.3 ** 2, 0.5 ** 2, 0.5 ** 2])
        self.t = t
        self.rejected_in_row = 0

    def predict(self, t):
        dt = max(t - self.t, 0.0)
        self.t = t
        if dt == 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.q
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = q * np.array([
            [dt4 / 4, 0, dt3 / 2, 0],
            [0, dt4 / 4, 0, dt3 / 2],
            [dt3 / 2, 0, dt2, 0],
            [0, dt3 / 2, 0, dt2],
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, anchor, r):
        d = self.x[:2] - anchor
        pred = float(np.linalg.norm(d))
        if pred < 1e-6:
            return False
        H = np.array([d[0] / pred, d[1] / pred, 0.0, 0.0])
        S = float(H @ self.P @ H) + self.r_var
        y = r - pred
        if y * y / S > self.GATE ** 2:
            self.rejected_in_row += 1
            return False
        self.rejected_in_row = 0
        K = (self.P @ H) / S
        self.x = self.x + K * y
        self.P = (np.eye(4) - np.outer(K, H)) @ self.P
        return True


class Tracker:
    # A least-squares fix counts as clean when its rms range residual is below this.
    CLEAN_FIX_RMS = 0.25
    # Restart the filter after this many clean fixes in a row disagree with it.
    DIVERGED_FIXES = 4

    def __init__(self, anchors, range_sigma, accel_sigma, raw=False):
        self.anchors = anchors
        self.raw = raw  # no filter: the position is the least-squares fix itself
        self.last_fix = None  # (pos, rms)
        self.range_sigma = range_sigma
        self.ekf = RangeEKF(range_sigma, accel_sigma)
        self.disagree = 0
        self.n_restarts = 0
        self.last = {}  # tag -> (t, corrected range)
        self.fixes = deque(maxlen=60)
        self.track = deque(maxlen=300)
        self.n_used = self.n_rejected = self.n_unknown = 0
        self.centre = np.mean([a["pos"] for a in anchors.values()], axis=0)

    def add(self, t, tag, r_raw, compensated=False):
        a = self.anchors.get(tag)
        if a is None:
            self.n_unknown += 1
            return
        r = r_raw if compensated else r_raw - a["bias"]
        self.last[tag] = (t, r)

        # Raw fix from the latest range of every anchor heard in the last 1.5 s.
        fresh = [(self.anchors[k]["pos"], v[1]) for k, v in self.last.items() if t - v[0] < 1.5]
        if self.raw:
            if len(fresh) >= 3:
                guess = self.last_fix[0] if self.last_fix is not None else self.centre
                fix = least_squares_fix(np.array([f[0] for f in fresh]), np.array([f[1] for f in fresh]), guess)
                if fix is not None:
                    self.last_fix = fix
                    self.fixes.append(fix[0])
                    self.track.append(fix[0].copy())
                    self.n_used += 1
            return

        if len(fresh) >= 3:
            guess = self.ekf.x[:2] if self.ekf.x is not None else self.centre
            fix = least_squares_fix(np.array([f[0] for f in fresh]), np.array([f[1] for f in fresh]), guess)
            if fix is not None:
                p, rms = fix
                self.fixes.append(p)
                if self.ekf.x is None:
                    self.ekf.init(p, t)
                elif rms < self.CLEAN_FIX_RMS:
                    # The filter can lock onto a wrong spot and still accept some
                    # ranges there; clean fixes that keep disagreeing reveal it.
                    sigma = math.sqrt(max(np.linalg.eigvalsh(self.ekf.P[:2, :2])))
                    far = np.linalg.norm(p - self.ekf.x[:2]) > max(0.5, 3 * sigma + 2 * self.range_sigma)
                    self.disagree = self.disagree + 1 if far else 0
                if self.ekf.rejected_in_row >= 6 or self.disagree >= self.DIVERGED_FIXES:
                    self.ekf.init(p, t)
                    self.disagree = 0
                    self.n_restarts += 1

        if self.ekf.x is None:
            return
        self.ekf.predict(t)
        if self.ekf.update(a["pos"], r):
            self.n_used += 1
        else:
            self.n_rejected += 1
        self.track.append(self.ekf.x[:2].copy())


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def run_plot(tracker, q, snapshot=None, snapshot_after=15.0, map_size=None, ser=None, tag_info=None, fps=40):
    import matplotlib
    if snapshot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # 'f' is matplotlib's fullscreen key; here it steps the tag's round period.
    plt.rcParams["keymap.fullscreen"] = [k for k in plt.rcParams["keymap.fullscreen"] if k != "f"]
    tag_info = tag_info if tag_info is not None else {}
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle, Ellipse

    anchors = tracker.anchors
    pts = np.array([a["pos"] for a in anchors.values()])
    margin = 1.0
    fig, ax = plt.subplots(figsize=(13, 10))
    fig.subplots_adjust(left=0.06, right=0.98, bottom=0.06, top=0.95)
    ax.set_aspect("equal")
    if map_size:  # fixed square view centred on the anchors
        cx, cy = (pts.min(axis=0) + pts.max(axis=0)) / 2
        ax.set_xlim(cx - map_size / 2, cx + map_size / 2)
        ax.set_ylim(cy - map_size / 2, cy + map_size / 2)
    else:
        ax.set_xlim(pts[:, 0].min() - margin, pts[:, 0].max() + margin)
        ax.set_ylim(pts[:, 1].min() - margin, pts[:, 1].max() + margin)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("UWB tag: raw least-squares position (no filter)" if tracker.raw else "UWB tag: EKF position")

    colors = ["tab:blue", "tab:green", "tab:purple", "tab:orange", "tab:brown"]
    circles = {}
    for i, (tag, a) in enumerate(anchors.items()):
        c = colors[i % len(colors)]
        ax.plot(*a["pos"], marker="^", markersize=12, color=c, linestyle="none")
        ax.annotate(a["name"], a["pos"], textcoords="offset points", xytext=(8, 8), color=c)
        # Solid, opaque, light colour: dashes and alpha make it slower to blit.
        light = [0.65 + 0.35 * v for v in matplotlib.colors.to_rgb(c)]
        circ = Circle(a["pos"], 0.01, fill=False, edgecolor=light, linewidth=0.8)
        ax.add_patch(circ)
        circles[tag] = circ

    fix_dots, = ax.plot([], [], ".", color="0.6", markersize=4, label="least-squares fix (raw)")
    # No alpha on the trail: a translucent 300-point line is the slowest artist to blit.
    trail, = ax.plot([], [], "-", color="#e8807f", linewidth=1.5,
                     label="raw track" if tracker.raw else "EKF track")
    tag_dot, = ax.plot([], [], "o", color="tab:red", markersize=11, label="tag (raw)" if tracker.raw else "tag (EKF)")
    ell = Ellipse((0, 0), 0, 0, fill=False, color="tab:red", alpha=0.6)
    ax.add_patch(ell)
    # Info box: a Tk label over the map when there is a window. A matplotlib text box
    # costs ~70 ms per draw (8x all the other artists), which capped the map at ~6 fps.
    info_label = None
    if not snapshot:
        try:
            import tkinter as tk
            info_label = tk.Label(fig.canvas.get_tk_widget(), text="", font=("Consolas", 9), justify="left",
                                  anchor="nw", bg="white", relief="solid", borderwidth=1, padx=6, pady=4)
            info_label.place(x=70, y=45)
        except Exception:
            info_label = None
    info = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", family="monospace", fontsize=9,
                   bbox=dict(boxstyle="round", fc="white", alpha=0.8), visible=info_label is None)
    last_info = {"t": 0.0, "text": None}
    ax.legend(loc="lower right", fontsize=8)

    def on_key(event):
        if event.key == "f" and ser is not None:
            ser.write(b"f")

    fig.canvas.mpl_connect("key_press_event", on_key)

    def drain():
        n = 0
        while True:
            try:
                t, tag, r, comp = q.get_nowait()
            except queue.Empty:
                return n
            tracker.add(t, tag, r, comp)
            n += 1

    def frame(_):
        drain()
        for tag, circ in circles.items():
            if tag in tracker.last:
                circ.set_radius(max(tracker.last[tag][1], 0.01))
        if tracker.fixes:
            f = np.array(tracker.fixes)
            fix_dots.set_data(f[:, 0], f[:, 1])
        lines = []
        if tracker.raw:
            if tracker.last_fix is not None:
                tr = np.array(tracker.track)
                trail.set_data(tr[:, 0], tr[:, 1])
                (x, y), rms = tracker.last_fix
                tag_dot.set_data([x], [y])
                lines.append(f"x {x:6.2f} m   y {y:6.2f} m   rms {rms:4.2f} m")
            else:
                lines.append("waiting for ranges from 3 anchors ...")
        elif tracker.ekf.x is not None:
            tr = np.array(tracker.track)
            trail.set_data(tr[:, 0], tr[:, 1])
            x, y = tracker.ekf.x[:2]
            tag_dot.set_data([x], [y])
            # 2-sigma position uncertainty ellipse.
            vals, vecs = np.linalg.eigh(tracker.ekf.P[:2, :2])
            vals = np.maximum(vals, 1e-9)
            ell.set_center((x, y))
            ell.width, ell.height = 4 * np.sqrt(vals)
            ell.angle = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
            speed = float(np.hypot(*tracker.ekf.x[2:]))
            lines.append(f"x {x:6.2f} m   y {y:6.2f} m   v {speed:4.2f} m/s")
        else:
            lines.append("waiting for ranges from 3 anchors ...")
        hz = tag_info.get("hz", {})
        for tag, a in anchors.items():
            rate = f"  {hz[tag]:4.1f} Hz" if tag in hz else ""
            if tag in tracker.last:
                lines.append(f"{a['name']:<16} r {tracker.last[tag][1]:5.2f} m{rate}")
            else:
                lines.append(f"{a['name']:<16} r   --{rate}")
        if "period" in tag_info:
            rnd = tag_info.get("round")
            rnd = f"{rnd:.1f} ms" if rnd is not None else "(measuring, 5 s)"
            lines.append(f"tag period {tag_info['period']} ms  round {rnd}  [f = change]")
        if tracker.raw:
            lines.append(f"fixes {tracker.n_used}")
        else:
            lines.append(f"ranges used {tracker.n_used}  rejected {tracker.n_rejected}  restarts {tracker.n_restarts}")
        text = "\n".join(lines)
        now = time.monotonic()
        if info_label is not None:
            # 5 Hz is enough to read and keeps Tk work off most frames.
            if text != last_info["text"] and now - last_info["t"] > 0.2:
                info_label.config(text=text)
                last_info["t"], last_info["text"] = now, text
            return artists
        info.set_text(text)
        return artists + [info]

    artists = [*circles.values(), fix_dots, trail, tag_dot, ell]

    if snapshot:
        t_end = time.monotonic() + snapshot_after
        while time.monotonic() < t_end:
            frame(None)
            time.sleep(0.1)
        frame(None)
        fig.savefig(snapshot, dpi=110)
        print(f"saved {snapshot}")
        return
    # blit: redraw only the moving artists over a cached background (~9 ms instead
    # of ~160 ms for the whole maximized figure).
    # The timer waits `interval` after each frame is drawn, not between frame starts,
    # so leave room for the ~30 ms a maximized window takes to draw and blit.
    _anim = FuncAnimation(fig, frame, interval=max(1, int(1000 / fps) - 30), blit=info_label is not None,
                          cache_frame_data=False)
    try:  # maximize once shown: plt.show() deiconifies the Tk window back to normal size
        win = fig.canvas.manager.window
        win.after(300, lambda: win.state("zoomed"))
    except Exception:
        pass
    plt.show()


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the initiator, e.g. COM6")
    src.add_argument("--replay", help="log file written with --log")
    src.add_argument("--simulate", action="store_true", help="synthetic tag, no hardware")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--raw", action="store_true", help="show the raw least-squares fix, no EKF")
    ap.add_argument("--config", default=os.path.join(here, "anchors.json"))
    ap.add_argument("--log", help="append raw serial lines (with time) to this file")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    ap.add_argument("--snapshot", help="no window: run for --seconds, save the plot to this PNG and exit")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--fps", type=float, default=30.0, help="map redraw rate (default 30)")
    args = ap.parse_args()

    anchors, range_sigma, accel_sigma, map_size, iir_alpha = load_config(args.config)
    q = queue.Queue()
    ser = None
    tag_info = {}
    stop = threading.Event()
    if args.port:
        ser = open_serial(args.port, args.baud)
        target, targs = serial_source, (ser, q, anchors, args.log, stop, tag_info, iir_alpha)
    elif args.replay:
        target, targs = replay_source, (args.replay, q, args.speed, stop)
    else:
        target, targs = simulate_source, (anchors, range_sigma, q, stop)
    threading.Thread(target=target, args=targs, daemon=True).start()

    tracker = Tracker(anchors, range_sigma, accel_sigma, raw=args.raw)
    try:
        run_plot(tracker, q, snapshot=args.snapshot, snapshot_after=args.seconds, map_size=map_size,
                 ser=ser, tag_info=tag_info, fps=args.fps)
    finally:
        stop.set()


if __name__ == "__main__":
    main()
