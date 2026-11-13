"""
Live dashboard / simulator for the STM32 custom-scheduler project.

Reads the telemetry lines from the serial port (same as PuTTY) and shows:
  - a top-down "car" view: heading from the gyro, ultrasonic beam, obstacle
  - live graphs: distance, yaw rate, accelerometer, scheduler jitter

Usage:
    python dashboard.py                    # real data from COM5
    python dashboard.py --port COM7        # different port
    python dashboard.py --demo             # fake data, no board needed
    python dashboard.py --log run1.csv     # also save every sample to CSV

Close PuTTY first: only one program can open COM5 at a time.
Keys: r = reset the car heading to straight ahead.

Install once:  pip install pyserial matplotlib numpy
"""
import argparse
import collections
import csv
import math
import random
import re
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

# ---------------------------------------------------------------- settings
BAUD = 115200
HISTORY_S = 30            # seconds of history in the graphs
OBSTACLE_CM = 20          # same as obstacle_near in the firmware
SKID_DPS = 30.0           # placeholder skid threshold on |yaw rate| (deg/s)
GYRO_DEADBAND_DPS = 0.5   # ignore tiny gyro noise so heading doesn't creep
MAX_RANGE_CM = 250        # distances above this are drawn as "nothing in range"

# Matches the Task_Telemetry line from the firmware (Step 5D).
LINE_RE = re.compile(
    r"t=(\d+)\s+dist=(\d+)\s+imu=(\d+)\s+jit\[(\d+)\.\.(\d+)\]\s+rd=(\d+)us\s+"
    r"gz=(-?\d+)\s+ay=(-?\d+)\s+az=(-?\d+)\s+err=(\d+)\s+miss=(\d+)"
)
FIELDS = ["t_ms", "dist_cm", "imu_count", "jit_min_us", "jit_max_us",
          "rd_us", "gz_dps", "ay_mg", "az_mg", "i2c_err", "miss"]

samples = collections.deque()       # filled by the reader thread
stop_flag = threading.Event()


def parse_line(line):
    m = LINE_RE.search(line)
    if not m:
        return None
    v = [int(x) for x in m.groups()]
    return {
        "t_ms": v[0], "dist_cm": v[1], "imu_count": v[2],
        "jit_min_us": v[3], "jit_max_us": v[4], "rd_us": v[5],
        "gz_dps": v[6] / 10.0,          # firmware sends tenths of deg/s
        "ay_mg": v[7], "az_mg": v[8], "i2c_err": v[9], "miss": v[10],
    }


def serial_reader(port, log_path):
    import serial  # pyserial
    log_file = open(log_path, "w", newline="") if log_path else None
    writer = csv.DictWriter(log_file, fieldnames=FIELDS) if log_file else None
    if writer:
        writer.writeheader()
    try:
        ser = serial.Serial(port, BAUD, timeout=0.5)
    except Exception as e:
        print(f"Could not open {port}: {e}\n"
              f"Is PuTTY (or another program) still using it?")
        stop_flag.set()
        return
    print(f"Reading {port} at {BAUD} baud...")
    while not stop_flag.is_set():
        try:
            raw = ser.readline()
        except Exception as e:
            print(f"Serial error: {e}")
            break
        if not raw:
            continue
        s = parse_line(raw.decode("ascii", errors="ignore"))
        if s:
            samples.append(s)
            if writer:
                writer.writerow(s)
                log_file.flush()
    ser.close()
    if log_file:
        log_file.close()


def demo_reader():
    """Fake data: a car that sees an obstacle approach and does a turn."""
    t = 0
    count = 0
    start = time.time()
    while not stop_flag.is_set():
        time.sleep(0.1)
        t += 100
        count += 50
        ph = (time.time() - start) % 24
        # obstacle slides in and out
        if ph < 6:
            dist = 999
        elif ph < 12:
            dist = int(200 - (ph - 6) * 30)
        elif ph < 14:
            dist = int(20 - (ph - 12) * 5)
        else:
            dist = int(10 + (ph - 14) * 20)
            dist = 999 if dist > 240 else dist
        # a turn between 14 s and 18 s
        gz = 45 * math.sin((ph - 14) / 4 * math.pi) if 14 < ph < 18 else 0.0
        gz += random.uniform(-0.5, 0.5)
        samples.append({
            "t_ms": t, "dist_cm": dist, "imu_count": count,
            "jit_min_us": 1993, "jit_max_us": 2005 + random.randint(0, 3),
            "rd_us": 403, "gz_dps": gz,
            "ay_mg": int(random.gauss(0, 6) + gz * 4),
            "az_mg": int(1000 + random.gauss(0, 8)),
            "i2c_err": 0, "miss": 0,
        })


# ------------------------------------------------------------------ GUI
def car_polygon(x, y, heading_deg, length=24, width=14):
    """Corner points of the car rectangle; heading 0 = pointing up (+y)."""
    pts = np.array([[-width / 2, -length / 2], [width / 2, -length / 2],
                    [width / 2, length / 2], [-width / 2, length / 2]])
    a = math.radians(heading_deg)   # positive yaw = counter-clockwise on screen
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return (pts @ rot.T) + np.array([x, y])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--demo", action="store_true", help="fake data, no board")
    ap.add_argument("--log", default=None, help="save samples to this CSV file")
    args = ap.parse_args()

    target = demo_reader if args.demo else (lambda: serial_reader(args.port, args.log))
    threading.Thread(target=target, daemon=True).start()

    maxn = HISTORY_S * 10 + 20          # telemetry arrives at 10 Hz
    H = {k: collections.deque(maxlen=maxn) for k in
         ["t", "dist", "gz", "ay", "azdev", "jmin", "jmax"]}
    state = {"yaw": 0.0, "last_t": None, "last": None}

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(13, 7.5))
    fig.canvas.manager.set_window_title("STM32 scheduler simulator")
    gs = fig.add_gridspec(4, 2, width_ratios=[1.1, 1.4], hspace=0.55, wspace=0.18)

    # ---- car view
    axc = fig.add_subplot(gs[:, 0])
    axc.set_xlim(-90, 90)
    axc.set_ylim(-40, MAX_RANGE_CM + 20)
    axc.set_aspect("equal")
    axc.set_title("Car view (top-down)")
    axc.set_xlabel("cm")
    axc.grid(alpha=0.2)
    for r in (50, 100, 150, 200, 250):
        axc.add_patch(plt.Circle((0, 0), r, fill=False, ls=":", color="gray", alpha=0.3))
        axc.text(3, r + 2, f"{r} cm", color="gray", fontsize=7)
    danger = plt.Circle((0, 0), OBSTACLE_CM, fill=True, color="red", alpha=0.12)
    axc.add_patch(danger)
    car = Polygon(car_polygon(0, 0, 0), closed=True, fc="#4aa3ff", ec="white", zorder=5)
    axc.add_patch(car)
    beam, = axc.plot([], [], "-", color="yellow", lw=1.5, zorder=4)
    obst, = axc.plot([], [], "s", ms=14, color="lime", zorder=6)
    status = axc.text(0.02, 0.98, "", transform=axc.transAxes, va="top",
                      fontsize=9, family="monospace",
                      bbox=dict(fc="black", ec="gray", alpha=0.7))
    banner = axc.text(0.5, 0.06, "", transform=axc.transAxes, ha="center",
                      fontsize=15, weight="bold")

    # ---- graphs
    ax1 = fig.add_subplot(gs[0, 1])
    ax2 = fig.add_subplot(gs[1, 1], sharex=ax1)
    ax3 = fig.add_subplot(gs[2, 1], sharex=ax1)
    ax4 = fig.add_subplot(gs[3, 1], sharex=ax1)

    ax1.set_ylabel("dist (cm)")
    ax1.set_ylim(0, MAX_RANGE_CM + 10)
    ax1.axhline(OBSTACLE_CM, color="red", ls="--", lw=1)
    l_dist, = ax1.plot([], [], color="lime")

    ax2.set_ylabel("yaw (deg/s)")
    ax2.axhline(SKID_DPS, color="orange", ls="--", lw=1)
    ax2.axhline(-SKID_DPS, color="orange", ls="--", lw=1)
    l_gz, = ax2.plot([], [], color="cyan")

    ax3.set_ylabel("accel (mg)")
    l_ay, = ax3.plot([], [], color="magenta", label="ay")
    l_az, = ax3.plot([], [], color="white", lw=0.8, label="az - 1000")
    ax3.legend(loc="upper left", fontsize=7)

    ax4.set_ylabel("IMU period (us)")
    ax4.set_xlabel("time (s)")
    ax4.set_ylim(1900, 2100)
    ax4.axhline(2000, color="gray", ls=":", lw=1)
    l_jmin, = ax4.plot([], [], color="yellow", label="min")
    l_jmax, = ax4.plot([], [], color="orange", label="max")
    ax4.legend(loc="upper left", fontsize=7)
    for a in (ax1, ax2, ax3):
        a.tick_params(labelbottom=False)
        a.grid(alpha=0.2)
    ax4.grid(alpha=0.2)

    def on_key(ev):
        if ev.key == "r":
            state["yaw"] = 0.0
    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        got = False
        while samples:
            s = samples.popleft()
            got = True
            # integrate the gyro for heading (use the board's own clock)
            if state["last_t"] is not None:
                dt = (s["t_ms"] - state["last_t"]) / 1000.0
                if 0 < dt < 1.0:
                    g = s["gz_dps"]
                    if abs(g) < GYRO_DEADBAND_DPS:
                        g = 0.0
                    state["yaw"] += g * dt
            elif s["t_ms"] == 0:
                pass
            if state["last_t"] is not None and s["t_ms"] < state["last_t"]:
                state["yaw"] = 0.0   # board was reset
                for d in H.values():
                    d.clear()
            state["last_t"] = s["t_ms"]
            state["last"] = s
            H["t"].append(s["t_ms"] / 1000.0)
            H["dist"].append(min(s["dist_cm"], MAX_RANGE_CM) if s["dist_cm"] < 999 else np.nan)
            H["gz"].append(s["gz_dps"])
            H["ay"].append(s["ay_mg"])
            H["azdev"].append(s["az_mg"] - 1000)
            H["jmin"].append(s["jit_min_us"] if s["jit_min_us"] < 100000 else np.nan)
            H["jmax"].append(s["jit_max_us"])

        s = state["last"]
        if s is None:
            status.set_text("waiting for data...")
            return
        if not got:
            return

        # ---- car view
        yaw = state["yaw"]
        car.set_xy(car_polygon(0, 0, yaw))
        a = math.radians(yaw)
        dx, dy = -math.sin(a), math.cos(a)          # forward direction on screen
        near = s["dist_cm"] < OBSTACLE_CM
        in_range = s["dist_cm"] <= MAX_RANGE_CM
        reach = s["dist_cm"] if in_range else MAX_RANGE_CM
        beam.set_data([dx * 12, dx * reach], [dy * 12, dy * reach])
        beam.set_color("red" if near else "yellow")
        if in_range:
            obst.set_data([dx * reach], [dy * reach])
            obst.set_color("red" if near else "lime")
        else:
            obst.set_data([], [])
        danger.set_alpha(0.3 if near else 0.12)

        skid = abs(s["gz_dps"]) > SKID_DPS
        msgs = []
        if near:
            msgs.append("OBSTACLE NEAR")
        if skid:
            msgs.append("SKID?")
        banner.set_text("  ".join(msgs) if msgs else "clear")
        banner.set_color("red" if msgs else "lime")

        miss_bad = s["miss"] > 0 or s["i2c_err"] > 0
        status.set_text(
            f"t        {s['t_ms'] / 1000:8.1f} s\n"
            f"distance {('--' if not in_range else s['dist_cm']):>8} cm\n"
            f"yaw rate {s['gz_dps']:8.1f} deg/s\n"
            f"heading  {yaw:8.1f} deg\n"
            f"I2C read {s['rd_us']:8d} us\n"
            f"jitter   {s['jit_min_us'] if s['jit_min_us'] < 100000 else '--'}.."
            f"{s['jit_max_us']} us\n"
            f"misses   {s['miss']:8d}\n"
            f"i2c err  {s['i2c_err']:8d}"
            + ("\n** TIMING/I2C PROBLEM **" if miss_bad else "")
        )

        # ---- graphs
        t = np.array(H["t"])
        l_dist.set_data(t, np.array(H["dist"]))
        l_gz.set_data(t, np.array(H["gz"]))
        l_ay.set_data(t, np.array(H["ay"]))
        l_az.set_data(t, np.array(H["azdev"]))
        l_jmin.set_data(t, np.array(H["jmin"]))
        l_jmax.set_data(t, np.array(H["jmax"]))
        ax1.set_xlim(t[-1] - HISTORY_S, t[-1] + 0.5)
        for ax, keys in ((ax2, ["gz"]), (ax3, ["ay", "azdev"])):
            vals = np.concatenate([np.array(H[k]) for k in keys])
            lim = max(np.nanmax(np.abs(vals)) * 1.2, SKID_DPS * 1.5 if ax is ax2 else 50)
            ax.set_ylim(-lim, lim)
        top = max(2100, float(np.nanmax(np.array(H["jmax"]))) + 20)
        ax4.set_ylim(1900, top)

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop_flag.set()
    return anim


if __name__ == "__main__":
    main()