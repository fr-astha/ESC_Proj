"""
Live dashboard / simulator for the STM32 custom-scheduler project.

Reads the telemetry lines from the serial port (same as PuTTY) and shows:
  - a top-down "car" view: heading from the gyro, ultrasonic beam, obstacle
  - the FIRMWARE's own decisions: skid flag (red car) and sensor-lost (grey car)
  - live graphs: distance, yaw rate, accelerometer, scheduler jitter
    (red band = firmware skid, orange band = IMU data not valid)

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
SKID_DPS = 30.0           # firmware SKID_ON_DPS_X10 / 10; drawn as a reference line,
                          # and used as a fallback if the firmware sends no skid= field
GYRO_DEADBAND_DPS = 0.5   # ignore tiny gyro noise so heading doesn't creep
MAX_RANGE_CM = 250        # distances above this are drawn as "nothing in range"
JIT_PLOT_MAX_US = 3000    # clip the jitter graph here (a 42 ms stall would flatten it)
NO_DATA_S = 1.5           # no telemetry line for this long = "NO TELEMETRY"

COL_OK, COL_SKID, COL_LOST = "#4aa3ff", "#ff4040", "#707070"

# Matches the Task_Telemetry line. The last four fields are optional so that
# older firmware lines (without skid/ev/rst/v) still parse.
LINE_RE = re.compile(
    r"t=(\d+)\s+dist=(\d+)\s+imu=(\d+)\s+jit\[(\d+)\.\.(\d+)\]\s+rd=(\d+)us\s+"
    r"gz=(-?\d+)\s+ay=(-?\d+)\s+az=(-?\d+)\s+err=(\d+)\s+miss=(\d+)"
    r"(?:\s+skid=(\d+)\s+ev=(\d+))?(?:\s+rst=(\d+))?(?:\s+v=(\d+))?"
)
FIELDS = ["t_ms", "dist_cm", "imu_count", "jit_min_us", "jit_max_us",
          "rd_us", "gz_dps", "ay_mg", "az_mg", "i2c_err", "miss",
          "skid", "ev", "rst", "valid"]

samples = collections.deque()       # filled by the reader thread
stop_flag = threading.Event()


def parse_line(line):
    m = LINE_RE.search(line)
    if not m:
        return None
    g = m.groups()
    v = [int(x) for x in g[:11]]
    return {
        "t_ms": v[0], "dist_cm": v[1], "imu_count": v[2],
        "jit_min_us": v[3], "jit_max_us": v[4], "rd_us": v[5],
        "gz_dps": v[6] / 10.0,          # firmware sends tenths of deg/s
        "ay_mg": v[7], "az_mg": v[8], "i2c_err": v[9], "miss": v[10],
        "skid": int(g[11]) if g[11] is not None else -1,   # -1 = not reported
        "ev": int(g[12]) if g[12] is not None else 0,
        "rst": int(g[13]) if g[13] is not None else 0,
        "valid": int(g[14]) if g[14] is not None else 1,
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
    ser.reset_input_buffer()
    ser.readline()                  # drop the first, usually half-received, line
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
    """Fake data (30 s loop): obstacle approaches, a skid, then a sensor dropout.
    The values are made up; the real board will not reset on every dropout."""
    t = count = err = miss = ev = rst = drops = 0
    skid = 0
    jmax, rdmax = 2005, 403
    last = (0.0, 0, 1000)               # last good gz, ay, az
    was_lost = False
    start = time.time()
    while not stop_flag.is_set():
        time.sleep(0.1)
        t += 100
        count += 50
        ph = (time.time() - start) % 30
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
        lost = 22 <= ph < 24.5          # simulated I2C dropout
        if not lost:
            gz = 45 * math.sin((ph - 14) / 4 * math.pi) if 14 < ph < 18 else 0.0
            gz += random.uniform(-0.5, 0.5)
            ay = int(random.gauss(0, 6) + gz * 4)
            az = int(1000 + random.gauss(0, 8))
            last = (gz, ay, az)
            # same hysteresis as the firmware: on above 30, off below 20 deg/s
            if not skid and abs(gz) > 30:
                skid = 1
                ev += 1
            elif skid and abs(gz) < 20:
                skid = 0
            if was_lost:
                drops += 1
                if drops % 2 == 0:
                    rst += 1
        else:
            gz, ay, az = last           # stale values, like the real firmware
            err += random.randint(6, 9)
            miss += random.randint(0, 1)
            jmax = max(jmax, 25999)
            rdmax = max(rdmax, 25383)
        was_lost = lost
        samples.append({
            "t_ms": t, "dist_cm": dist, "imu_count": count,
            "jit_min_us": 1993, "jit_max_us": max(jmax, 2000 + random.randint(0, 5)),
            "rd_us": rdmax, "gz_dps": gz, "ay_mg": ay, "az_mg": az,
            "i2c_err": err, "miss": miss,
            "skid": skid, "ev": ev, "rst": rst, "valid": 0 if lost else 1,
        })


# ------------------------------------------------------------------ GUI
def car_polygon(x, y, heading_deg, length=24, width=14):
    """Corner points of the car rectangle; heading 0 = pointing up (+y)."""
    pts = np.array([[-width / 2, -length / 2], [width / 2, -length / 2],
                    [width / 2, length / 2], [-width / 2, length / 2]])
    a = math.radians(heading_deg)   # positive yaw = counter-clockwise on screen
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return (pts @ rot.T) + np.array([x, y])


def is_skid(s):
    """Use the firmware's decision when it is reported, else a plain threshold."""
    if s["skid"] >= 0:
        return bool(s["skid"])
    return abs(s["gz_dps"]) > SKID_DPS


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
         ["t", "dist", "gz", "ay", "azdev", "jmin", "jmax", "skid", "valid"]}
    state = {"yaw": 0.0, "last_t": None, "last": None, "last_rx": time.time()}

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(13, 7.5))
    fig.canvas.manager.set_window_title("STM32 scheduler simulator")
    gs = fig.add_gridspec(4, 2, width_ratios=[1.1, 1.4], hspace=0.55, wspace=0.18)
    fig.text(0.99, 0.008, "red band = firmware skid     orange band = IMU data not valid",
             ha="right", fontsize=8, color="gray")

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
    car = Polygon(car_polygon(0, 0, 0), closed=True, fc=COL_OK, ec="white", zorder=5)
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
    l_jmax, = ax4.plot([], [], color="orange", label="max (clipped at 3000)")
    ax4.legend(loc="upper left", fontsize=7)
    for a in (ax1, ax2, ax3):
        a.tick_params(labelbottom=False)
        a.grid(alpha=0.2)
    ax4.grid(alpha=0.2)

    shading = []                         # band artists, redrawn every frame

    def redraw_shading(t):
        for a in shading:
            a.remove()
        shading.clear()
        lost = np.array(H["valid"]) == 0
        sk = np.array(H["skid"]) == 1
        for ax in (ax2, ax3, ax4):
            if lost.any():
                shading.append(ax.fill_between(
                    t, 0, 1, where=lost, step="mid", lw=0, color="orange", alpha=0.25,
                    transform=ax.get_xaxis_transform()))
        if sk.any():
            shading.append(ax2.fill_between(
                t, 0, 1, where=sk, step="mid", lw=0, color="red", alpha=0.25,
                transform=ax2.get_xaxis_transform()))

    def on_key(ev):
        if ev.key == "r":
            state["yaw"] = 0.0
    fig.canvas.mpl_connect("key_press_event", on_key)

    def set_car_view(color, text, text_color):
        car.set_facecolor(color)
        banner.set_text(text)
        banner.set_color(text_color)

    def update(_):
        got = False
        while samples:
            s = samples.popleft()
            got = True
            if state["last_t"] is not None and s["t_ms"] < state["last_t"]:
                state["yaw"] = 0.0           # board was reset: start fresh
                state["last_t"] = None
                for d in H.values():
                    d.clear()
            # integrate the gyro for heading, but only from valid (fresh) readings
            if state["last_t"] is not None and s["valid"]:
                dt = (s["t_ms"] - state["last_t"]) / 1000.0
                if 0 < dt < 1.0:
                    g = s["gz_dps"]
                    if abs(g) < GYRO_DEADBAND_DPS:
                        g = 0.0
                    state["yaw"] += g * dt
            state["last_t"] = s["t_ms"]
            state["last"] = s
            state["last_rx"] = time.time()
            ok = bool(s["valid"])
            H["t"].append(s["t_ms"] / 1000.0)
            H["dist"].append(min(s["dist_cm"], MAX_RANGE_CM) if s["dist_cm"] < 999 else np.nan)
            # stale IMU values are drawn as gaps, not as flat lines
            H["gz"].append(s["gz_dps"] if ok else np.nan)
            H["ay"].append(s["ay_mg"] if ok else np.nan)
            H["azdev"].append(s["az_mg"] - 1000 if ok else np.nan)
            H["jmin"].append(s["jit_min_us"] if s["jit_min_us"] < 100000 else np.nan)
            H["jmax"].append(s["jit_max_us"])
            H["skid"].append(1 if (ok and is_skid(s)) else 0)
            H["valid"].append(1 if ok else 0)

        s = state["last"]
        if s is None:
            status.set_text("waiting for data...")
            return
        if not got:
            if time.time() - state["last_rx"] > NO_DATA_S:
                set_car_view(COL_LOST, "NO TELEMETRY", "gray")
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

        valid = bool(s["valid"])
        skid = valid and is_skid(s)
        msgs = []
        if not valid:
            msgs.append("SENSOR LOST")
        elif skid:
            msgs.append("SKID")
        if near:
            msgs.append("OBSTACLE NEAR")
        if not valid:
            set_car_view(COL_LOST, "  ".join(msgs), "orange")
        elif skid:
            set_car_view(COL_SKID, "  ".join(msgs), "red")
        else:
            set_car_view(COL_OK, "  ".join(msgs) if msgs else "clear",
                         "red" if msgs else "lime")

        jmin = s["jit_min_us"] if s["jit_min_us"] < 100000 else "--"
        sk_txt = ("--" if s["skid"] < 0 else ("ON" if s["skid"] else "off"))
        trouble = s["miss"] > 0 or s["i2c_err"] > 0 or not valid
        status.set_text(
            f"t         {s['t_ms'] / 1000:8.1f} s\n"
            f"distance  {('--' if not in_range else s['dist_cm']):>8} cm\n"
            f"yaw rate  {s['gz_dps']:8.1f} deg/s{'' if valid else '  (stale)'}\n"
            f"heading   {yaw:8.1f} deg\n"
            f"IMU valid {'yes' if valid else 'NO':>8}\n"
            f"skid      {sk_txt:>8}   events {s['ev']}\n"
            f"I2C read  {s['rd_us']:8d} us (worst)\n"
            f"jitter    {jmin}..{s['jit_max_us']} us\n"
            f"misses    {s['miss']:8d}\n"
            f"i2c err   {s['i2c_err']:8d}\n"
            f"IMU resets{s['rst']:8d}"
            + ("\n(counters are since boot)" if trouble else "")
        )

        # ---- graphs
        t = np.array(H["t"])
        l_dist.set_data(t, np.array(H["dist"]))
        l_gz.set_data(t, np.array(H["gz"]))
        l_ay.set_data(t, np.array(H["ay"]))
        l_az.set_data(t, np.array(H["azdev"]))
        jmin_a = np.array(H["jmin"], dtype=float)
        jmax_a = np.array(H["jmax"], dtype=float)
        l_jmin.set_data(t, np.minimum(jmin_a, JIT_PLOT_MAX_US))
        l_jmax.set_data(t, np.minimum(jmax_a, JIT_PLOT_MAX_US))
        ax1.set_xlim(t[-1] - HISTORY_S, t[-1] + 0.5)
        for ax, keys, floor in ((ax2, ["gz"], SKID_DPS * 1.5), (ax3, ["ay", "azdev"], 50)):
            vals = np.concatenate([np.array(H[k], dtype=float) for k in keys])
            vals = vals[~np.isnan(vals)]
            peak = float(np.abs(vals).max()) if vals.size else 0.0
            lim = max(peak * 1.2, floor)
            ax.set_ylim(-lim, lim)
        top = min(max(2100.0, float(np.nanmax(jmax_a)) + 20), JIT_PLOT_MAX_US)
        lo = 1900.0
        if not np.all(np.isnan(jmin_a)):
            lo = max(0.0, min(1900.0, float(np.nanmin(jmin_a)) - 50))
        ax4.set_ylim(lo, top)
        redraw_shading(t)

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop_flag.set()
    return anim


if __name__ == "__main__":
    main()