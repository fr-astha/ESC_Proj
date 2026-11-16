"""
Hardware-in-the-loop simulator for the STM32 custom-scheduler project.

REAL parts (from the board, over the serial port):
  - ultrasonic distance      -> the obstacle in front of the virtual car
  - IMU yaw rate             -> turn / skid (you cause it by rotating the board)
  - the STM32's own decisions: skid flag, throttle cut, obstacle stop,
    sensor-fault stop, left/right wheel commands, steering correction
VIRTUAL parts (on this PC):
  - the car: it tries to reach the speed you set with the sliders, limited by
    the acceleration slider, and it obeys the firmware (cut / stop).
    The physics are deliberately simple: a speed ramp and a braking distance.

Views: side view (speed, acceleration, obstacle, skid marks), top-down view
(heading, wheel commands, correction), and live graphs.

Usage:
    python dashboard.py                    # real data from COM5
    python dashboard.py --port COM7        # different port
    python dashboard.py --demo             # fake data, no board needed
    python dashboard.py --log run1.csv     # also save every sample to CSV

Close PuTTY first: only one program can open COM5 at a time.
Keys: r = reset the heading,  c = clear a collision.

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
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle, Polygon, Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.widgets import Slider

# ---------------------------------------------------------------- settings
BAUD = 115200
HISTORY_S = 30            # seconds of history in the graphs
OBSTACLE_CM = 20          # same as obstacle_near in the firmware
SKID_DPS = 30.0           # firmware SKID_ON_DPS_X10 / 10 (reference line / fallback)
GYRO_DEADBAND_DPS = 0.5   # ignore tiny gyro noise so heading doesn't creep
MAX_RANGE_CM = 150        # top-down view range
JIT_PLOT_MAX_US = 3000    # clip the jitter graph here (a 42 ms stall would flatten it)
NO_DATA_S = 1.5           # no telemetry line for this long = "NO TELEMETRY"

# virtual car. Keep the two marked values equal to the firmware's.
VMAX_CMS = 100.0          # speed at slider = 100 %
BRAKE_CMS2 = 400.0        # braking deceleration
COLLIDE_CM = 3            # obstacle this close while moving = collision
SKID_THROTTLE = 0.5       # = SKID_THROTTLE_PCT / 100 in the firmware
BASE_SPEED_PCT = 60       # = BASE_SPEED_PCT in the firmware
OBST_VISIBLE_CM = 100     # side view shows obstacles up to this far ahead
CAR_L = 24.0              # car length in the side view (cm)

COL_OK, COL_SKID, COL_LOST, COL_STOP = "#4aa3ff", "#ff4040", "#707070", "#e0a030"
ST_NAMES = {0: "RUNNING", 1: "SKID - THROTTLE CUT", 2: "OBSTACLE - STOPPING",
            3: "SENSOR FAULT - STOPPED"}

# Matches the Task_Telemetry line. Everything after miss= is optional, so older
# firmware lines (without skid/ev/rst/v or L/R/c/st) still parse.
LINE_RE = re.compile(
    r"t=(\d+)\s+dist=(\d+)\s+imu=(\d+)\s+jit\[(\d+)\.\.(\d+)\]\s+rd=(\d+)us\s+"
    r"gz=(-?\d+)\s+ay=(-?\d+)\s+az=(-?\d+)\s+err=(\d+)\s+miss=(\d+)"
    r"(?:\s+skid=(\d+)\s+ev=(\d+))?(?:\s+rst=(\d+))?(?:\s+v=(\d+))?"
    r"(?:\s+L=(-?\d+)\s+R=(-?\d+)\s+c=(-?\d+)\s+st=(\d+))?"
    r"(?:\s+tg=(\d+)\s+ac=(\d+)\s+rx=(\d+)\s+sp=(-?\d+))?"
)
FIELDS = ["t_ms", "dist_cm", "imu_count", "jit_min_us", "jit_max_us",
          "rd_us", "gz_dps", "ay_mg", "az_mg", "i2c_err", "miss",
          "skid", "ev", "rst", "valid", "cmd_l", "cmd_r", "corr", "st",
          "tg", "ac", "rx", "sp"]

samples = collections.deque()       # filled by the reader thread
stop_flag = threading.Event()
port_obj = [None]                   # the open serial port, used to send commands


def send_cmd(speed_pct, accel):
    """Send the slider values to the board as 'S<speed>,<accel>'."""
    p = port_obj[0]
    if p is None:
        return
    try:
        p.write(f"S{int(speed_pct)},{int(accel)}\n".encode("ascii"))
    except Exception as e:
        print(f"Serial write error: {e}")


def parse_line(line):
    m = LINE_RE.search(line)
    if not m:
        return None
    g = m.groups()
    v = [int(x) for x in g[:11]]

    def opt(i, default):
        return int(g[i]) if g[i] is not None else default

    return {
        "t_ms": v[0], "dist_cm": v[1], "imu_count": v[2],
        "jit_min_us": v[3], "jit_max_us": v[4], "rd_us": v[5],
        "gz_dps": v[6] / 10.0,          # firmware sends tenths of deg/s
        "ay_mg": v[7], "az_mg": v[8], "i2c_err": v[9], "miss": v[10],
        "skid": opt(11, -1),            # -1 = not reported by the firmware
        "ev": opt(12, 0), "rst": opt(13, 0), "valid": opt(14, 1),
        "cmd_l": opt(15, -1), "cmd_r": opt(16, -1), "corr": opt(17, 0),
        "st": opt(18, -1),
        "tg": opt(19, -1), "ac": opt(20, -1), "rx": opt(21, 0),
        "sp": opt(22, -1),              # board's own speed (cm/s), -1 = not reported
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
    port_obj[0] = ser
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


def ctrl_model(gz, skid, near, valid):
    """Python copy of the firmware's control_update(), used only by --demo."""
    if not valid:
        return 0, 0, 0, 3
    if near:
        return 0, 0, 0, 2
    base = BASE_SPEED_PCT * (SKID_THROTTLE if skid else 1.0)
    err = -int(gz * 10)                 # expected yaw is 0
    c = 0 if abs(err) <= 50 else int(err / 10)
    c = max(-40, min(40, c))
    left = int(max(0, min(100, base - c)))
    right = int(max(0, min(100, base + c)))
    return left, right, c, (1 if skid else 0)


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
        cl, cr, corr, st = ctrl_model(gz, skid, dist < OBSTACLE_CM, not lost)
        samples.append({
            "t_ms": t, "dist_cm": dist, "imu_count": count,
            "jit_min_us": 1993, "jit_max_us": max(jmax, 2000 + random.randint(0, 5)),
            "rd_us": rdmax, "gz_dps": gz, "ay_mg": ay, "az_mg": az,
            "i2c_err": err, "miss": miss,
            "skid": skid, "ev": ev, "rst": rst, "valid": 0 if lost else 1,
            "cmd_l": cl, "cmd_r": cr, "corr": corr, "st": st,
        })


# ---------------------------------------------------------------- virtual car
class Sim:
    """A very small virtual car. It tries to reach the slider speed (limited by
    the acceleration slider); the firmware's state can cut or stop it."""

    def __init__(self):
        self.v = 0.0                    # cm/s
        self.odo = 0.0                  # cm travelled
        self.acc = 0.0                  # smoothed acceleration, cm/s^2
        self.wheel = 0.0                # wheel angle, rad
        self.pitch = 0.0                # body pitch, degrees (nose up = +)
        self.crashed = False
        self.target_pct = 50.0          # speed slider
        self.accel = 120.0              # acceleration slider, cm/s^2
        self.skid_runs = []             # [start_odo, end_odo] of skid marks
        self.skid_open = False

    def target_speed(self, allowed):
        return self.target_pct / 100.0 * VMAX_CMS * (0.0 if self.crashed else allowed)

    def step(self, dt, allowed, dist_cm, skidding, board_v=None):
        target = self.target_speed(allowed)
        if board_v is not None and not self.crashed:   # the STM32 runs the ramp: just follow it
            dv = board_v - self.v
        elif target >= self.v:
            dv = min(target - self.v, self.accel * dt)
        else:
            dv = -min(self.v - target, BRAKE_CMS2 * dt)
        self.v = max(0.0, self.v + dv)
        a = dv / dt if dt > 1e-6 else 0.0
        self.acc += (a - self.acc) * min(1.0, dt * 8.0)
        self.odo += self.v * dt
        self.wheel += self.v * dt / 3.5
        self.pitch = float(np.clip(self.acc / 30.0, -8.0, 8.0))
        if dist_cm <= COLLIDE_CM and self.v > 5.0 and not self.crashed:
            self.crashed = True
            self.v = 0.0
        if skidding and self.v > 2.0:
            if not self.skid_open:
                self.skid_runs.append([self.odo, self.odo])
                self.skid_open = True
            self.skid_runs[-1][1] = self.odo
        else:
            self.skid_open = False
        self.skid_runs = [r for r in self.skid_runs if self.odo - r[1] < 60][-30:]

    def brake_distance(self):
        return self.v * self.v / (2.0 * BRAKE_CMS2)


def is_skid(s):
    """Use the firmware's decision when it is reported, else a plain threshold."""
    if s["skid"] >= 0:
        return bool(s["skid"])
    return abs(s["gz_dps"]) > SKID_DPS


def firmware_factor(s):
    """How much of the slider speed the firmware lets the car have (0..1)."""
    if not s["valid"]:
        return 0.0
    st = s["st"]
    if st < 0:                          # older firmware: work it out
        if s["dist_cm"] < OBSTACLE_CM:
            return 0.0
        return SKID_THROTTLE if is_skid(s) else 1.0
    return {0: 1.0, 1: SKID_THROTTLE}.get(st, 0.0)


def state_of(s, silent):
    """(name, colour) of what the car is doing."""
    if s is None or silent:
        return "NO TELEMETRY", COL_LOST
    if not s["valid"] or s["st"] == 3:
        return ST_NAMES[3], COL_LOST
    if s["st"] == 2 or (s["st"] < 0 and s["dist_cm"] < OBSTACLE_CM):
        return ST_NAMES[2], COL_STOP
    if s["st"] == 1 or (s["st"] < 0 and is_skid(s)):
        return ST_NAMES[1], COL_SKID
    return ST_NAMES[0], COL_OK


# ------------------------------------------------------------------ GUI
def car_polygon(x, y, heading_deg, length=24, width=14):
    """Corner points of the car rectangle (top-down); heading 0 = pointing up."""
    pts = np.array([[-width / 2, -length / 2], [width / 2, -length / 2],
                    [width / 2, length / 2], [-width / 2, length / 2]])
    a = math.radians(heading_deg)   # positive yaw = counter-clockwise on screen
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return (pts @ rot.T) + np.array([x, y])


BODY_PTS = [(0, 3), (0, 9), (3, 10.5), (7, 11), (10, 15), (18, 15), (21, 11), (24, 10), (24, 3)]
WIN_PTS = [(10.8, 11.6), (11.8, 14), (17.2, 14), (19.2, 11.6)]
WHEELS = [(5.0, 3.5), (19.0, 3.5)]
PITCH_CENTRE = (12.0, 7.0)


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
         ["t", "dist", "gz", "cl", "cr", "spd", "tspd", "jmin", "jmax", "skid", "valid"]}
    now0 = time.time()
    state = {"last_tx": 0.0, "yaw": 0.0, "last_t": None, "last": None, "last_rx": now0, "t_frame": now0}
    sim = Sim()

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(14, 9))
    fig.canvas.manager.set_window_title("STM32 scheduler simulator")
    gs = fig.add_gridspec(5, 2, width_ratios=[1.15, 1.5], left=0.05, right=0.985,
                          top=0.95, bottom=0.15, hspace=0.65, wspace=0.16)
    fig.text(0.99, 0.008, "red band = firmware skid     orange band = IMU data not valid",
             ha="right", fontsize=8, color="gray")

    # ---- side view
    axs = fig.add_subplot(gs[0:2, 0])
    axs.set_xlim(-15, 125)
    axs.set_ylim(-8, 42)
    axs.set_aspect("equal", adjustable="box")
    axs.set_title("Side view (virtual car, real sensors)")
    axs.set_yticks([])
    axs.set_xlabel("cm ahead of the car")
    axs.add_patch(Rectangle((-15, -8), 140, 8, fc="#2b2b2b", ec="none", zorder=1))
    road_lc = LineCollection([], colors="#777777", linewidths=1.5, zorder=2)
    skid_lc = LineCollection([], colors="black", linewidths=3, alpha=0.85, zorder=2)
    axs.add_collection(road_lc)
    axs.add_collection(skid_lc)
    body = Polygon(BODY_PTS, closed=True, fc=COL_OK, ec="white", zorder=5)
    win = Polygon(WIN_PTS, closed=True, fc="#cfe8ff", ec="none", zorder=6)
    wheel_patches = [Circle(c, 3.5, fc="#202020", ec="white", zorder=7) for c in WHEELS]
    spokes = LineCollection([], colors="white", linewidths=1.2, zorder=8)
    for p in [body, win] + wheel_patches:
        axs.add_patch(p)
    axs.add_collection(spokes)
    car_artists = [body, win, spokes] + wheel_patches
    obst = Rectangle((CAR_L + 50, 0), 6, 14, fc="lime", ec="white", zorder=4)
    axs.add_patch(obst)
    sbeam, = axs.plot([], [], ":", color="yellow", lw=1.3, zorder=3)
    side_info = axs.text(0.01, 0.97, "", transform=axs.transAxes, va="top", fontsize=8,
                         family="monospace", bbox=dict(fc="black", ec="gray", alpha=0.7))
    side_banner = axs.text(0.64, 0.92, "", transform=axs.transAxes, ha="center",
                           va="top", fontsize=11, weight="bold")

    # ---- top-down view
    axc = fig.add_subplot(gs[2:5, 0])
    axc.set_xlim(-70, 70)
    axc.set_ylim(-30, MAX_RANGE_CM + 10)
    axc.set_aspect("equal", adjustable="box")
    axc.set_title("Top-down view (heading and wheel commands)")
    axc.set_xlabel("cm")
    axc.grid(alpha=0.2)
    for r in (50, 100, 150):
        axc.add_patch(plt.Circle((0, 0), r, fill=False, ls=":", color="gray", alpha=0.3))
        axc.text(2, r + 2, f"{r} cm", color="gray", fontsize=7)
    danger = plt.Circle((0, 0), OBSTACLE_CM, fill=True, color="red", alpha=0.12)
    axc.add_patch(danger)
    car = Polygon(car_polygon(0, 0, 0), closed=True, fc=COL_OK, ec="white", zorder=5)
    axc.add_patch(car)
    beam, = axc.plot([], [], "-", color="yellow", lw=1.5, zorder=4)
    obst_top, = axc.plot([], [], "s", ms=12, color="lime", zorder=6)
    lbar = Rectangle((-62, 0), 9, 1, fc="#3ddc84", ec="white", zorder=3)
    rbar = Rectangle((53, 0), 9, 1, fc="#3ddc84", ec="white", zorder=3)
    axc.add_patch(lbar)
    axc.add_patch(rbar)
    for x0 in (-62, 53):                # dashed line = normal base speed
        axc.plot([x0, x0 + 9], [BASE_SPEED_PCT * 0.9] * 2, "--", color="gray", lw=1)
    ltxt = axc.text(-57.5, -9, "L", ha="center", fontsize=8)
    rtxt = axc.text(57.5, -9, "R", ha="center", fontsize=8)
    status = axc.text(0.02, 0.98, "", transform=axc.transAxes, va="top",
                      fontsize=8, family="monospace",
                      bbox=dict(fc="black", ec="gray", alpha=0.7))
    banner = axc.text(0.5, 0.05, "", transform=axc.transAxes, ha="center",
                      fontsize=13, weight="bold")

    # ---- graphs
    ax1 = fig.add_subplot(gs[0, 1])
    ax2 = fig.add_subplot(gs[1, 1], sharex=ax1)
    ax3 = fig.add_subplot(gs[2, 1], sharex=ax1)
    ax4 = fig.add_subplot(gs[3, 1], sharex=ax1)
    ax5 = fig.add_subplot(gs[4, 1], sharex=ax1)

    ax1.set_ylabel("dist (cm)")
    ax1.set_ylim(0, MAX_RANGE_CM + 10)
    ax1.axhline(OBSTACLE_CM, color="red", ls="--", lw=1)
    l_dist, = ax1.plot([], [], color="lime")

    ax2.set_ylabel("yaw (deg/s)")
    ax2.axhline(SKID_DPS, color="orange", ls="--", lw=1)
    ax2.axhline(-SKID_DPS, color="orange", ls="--", lw=1)
    l_gz, = ax2.plot([], [], color="cyan")

    ax3.set_ylabel("wheel cmd (%)")
    ax3.set_ylim(-5, 105)
    l_cl, = ax3.plot([], [], color="#3ddc84", label="left")
    l_cr, = ax3.plot([], [], color="magenta", label="right")
    ax3.legend(loc="upper left", fontsize=7, ncol=2)

    ax4.set_ylabel("speed (cm/s)")
    ax4.set_ylim(0, VMAX_CMS * 1.1)
    l_spd, = ax4.plot([], [], color="white", label="car")
    l_tspd, = ax4.plot([], [], color="gray", ls="--", label="allowed target")
    ax4.legend(loc="upper left", fontsize=7, ncol=2)

    ax5.set_ylabel("IMU period (us)")
    ax5.set_xlabel("time (s)")
    ax5.set_ylim(1900, 2100)
    ax5.axhline(2000, color="gray", ls=":", lw=1)
    l_jmin, = ax5.plot([], [], color="yellow", label="min")
    l_jmax, = ax5.plot([], [], color="orange", label="max (clipped at 3000)")
    ax5.legend(loc="upper left", fontsize=7)
    for a in (ax1, ax2, ax3, ax4):
        a.tick_params(labelbottom=False)
    for a in (ax1, ax2, ax3, ax4, ax5):
        a.grid(alpha=0.2)

    # ---- sliders
    sl_speed = Slider(fig.add_axes([0.11, 0.075, 0.30, 0.022]), "target speed %",
                      0, 100, valinit=sim.target_pct, valstep=1)
    sl_acc = Slider(fig.add_axes([0.11, 0.035, 0.30, 0.022]), "accel cm/s2",
                    20, 300, valinit=sim.accel, valstep=5)

    def on_speed(v):
        sim.target_pct = float(v)
        send_cmd(sim.target_pct, sim.accel)

    def on_acc(v):
        sim.accel = float(v)
        send_cmd(sim.target_pct, sim.accel)
    sl_speed.on_changed(on_speed)
    sl_acc.on_changed(on_acc)

    shading = []                         # band artists, redrawn every telemetry frame

    def redraw_shading(t):
        for a in shading:
            a.remove()
        shading.clear()
        lost = np.array(H["valid"]) == 0
        sk = np.array(H["skid"]) == 1
        for ax in (ax2, ax3, ax4, ax5):
            if lost.any():
                shading.append(ax.fill_between(
                    t, 0, 1, where=lost, step="mid", lw=0, color="orange", alpha=0.25,
                    transform=ax.get_xaxis_transform()))
        for ax in (ax2, ax3):
            if sk.any():
                shading.append(ax.fill_between(
                    t, 0, 1, where=sk, step="mid", lw=0, color="red", alpha=0.25,
                    transform=ax.get_xaxis_transform()))

    def on_key(ev):
        if ev.key == "r":
            state["yaw"] = 0.0
        elif ev.key == "c":
            sim.crashed = False
    fig.canvas.mpl_connect("key_press_event", on_key)

    def draw_side(s, allowed, silent, col, name):
        # car body follows the pitch (nose up when accelerating, down when braking)
        tf = Affine2D().rotate_deg_around(PITCH_CENTRE[0], PITCH_CENTRE[1], sim.pitch) + axs.transData
        for a in car_artists:
            a.set_transform(tf)
        body.set_facecolor(col)
        segs = []
        for cx, cy in WHEELS:
            for k in range(3):
                ang = -sim.wheel + k * math.pi / 3.0
                dx, dy = 3.2 * math.cos(ang), 3.2 * math.sin(ang)
                segs.append([(cx - dx, cy - dy), (cx + dx, cy + dy)])
        spokes.set_segments(segs)
        # road ticks and skid marks scroll backwards as the car moves
        off = sim.odo % 20.0
        road_lc.set_segments([[(k * 20.0 - off, -6.5), (k * 20.0 - off, -1.5)]
                              for k in range(-1, 8)])
        skid_lc.set_segments([[(WHEELS[0][0] - (sim.odo - r[0]), 0.4),
                               (WHEELS[0][0] - (sim.odo - r[1]), 0.4)]
                              for r in sim.skid_runs])
        # obstacle at the real ultrasonic distance
        dist = None if (s is None or silent) else s["dist_cm"]
        if dist is not None and dist <= OBST_VISIBLE_CM:
            obst.set_visible(True)
            obst.set_xy((CAR_L + dist, 0))
            obst.set_facecolor("red" if dist < OBSTACLE_CM else "lime")
            sbeam.set_data([CAR_L, CAR_L + dist], [8, 8])
        else:
            obst.set_visible(False)
            if dist is not None and dist <= MAX_RANGE_CM + 100:
                sbeam.set_data([CAR_L, 125], [8, 8])
            else:
                sbeam.set_data([], [])
        tgt = sim.target_speed(allowed)
        warn = ""
        if dist is not None and dist <= OBST_VISIBLE_CM and sim.v > 5 \
                and sim.brake_distance() > max(dist - COLLIDE_CM, 0):
            warn = "\n! cannot stop in time"
        side_info.set_text(
            f"speed    {sim.v:6.0f} cm/s\n"
            f"allowed  {tgt:6.0f} cm/s\n"
            f"accel    {sim.acc:+6.0f} cm/s2\n"
            f"brake d  {sim.brake_distance():6.1f} cm\n"
            f"obstacle {('--' if dist is None or dist > 400 else dist):>6} cm" + warn)
        if sim.crashed:
            side_banner.set_text("COLLISION  (press c)")
            side_banner.set_color("red")
        else:
            side_banner.set_text(name)
            side_banner.set_color(col if col != COL_OK else "lime")

    def update(_):
        now = time.time()
        dt = min(max(now - state["t_frame"], 0.0), 0.2)
        state["t_frame"] = now
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
                d_t = (s["t_ms"] - state["last_t"]) / 1000.0
                if 0 < d_t < 1.0:
                    g = s["gz_dps"]
                    if abs(g) < GYRO_DEADBAND_DPS:
                        g = 0.0
                    state["yaw"] += g * d_t
            state["last_t"] = s["t_ms"]
            state["last"] = s
            state["last_rx"] = now
            ok = bool(s["valid"])
            H["t"].append(s["t_ms"] / 1000.0)
            H["dist"].append(min(s["dist_cm"], MAX_RANGE_CM) if s["dist_cm"] < 999 else np.nan)
            H["gz"].append(s["gz_dps"] if ok else np.nan)     # stale values = gaps
            H["cl"].append(max(s["cmd_l"], 0))
            H["cr"].append(max(s["cmd_r"], 0))
            H["spd"].append(sim.v)
            H["tspd"].append(sim.target_speed(firmware_factor(s)))
            H["jmin"].append(s["jit_min_us"] if s["jit_min_us"] < 100000 else np.nan)
            H["jmax"].append(s["jit_max_us"])
            H["skid"].append(1 if (ok and is_skid(s)) else 0)
            H["valid"].append(1 if ok else 0)

        s = state["last"]
        silent = (now - state["last_rx"]) > NO_DATA_S
        if s is None or silent:
            allowed, dist, skidding = 0.0, 999, False   # no data = fail-safe stop
        else:
            allowed = firmware_factor(s)
            dist = s["dist_cm"]
            skidding = bool(s["valid"]) and is_skid(s)
        board_v = float(s["sp"]) if (s is not None and not silent and s["sp"] >= 0) else None
        if now - state["last_tx"] > 0.5:   # re-send often: a board reset forgets it
            state["last_tx"] = now
            send_cmd(sim.target_pct, sim.accel)
        sim.step(dt, allowed, dist, skidding, board_v)
        name, col = state_of(s, silent)
        draw_side(s, allowed, silent, col, name)

        if s is None:
            status.set_text("waiting for data...")
            return
        if not got:
            if silent:
                car.set_facecolor(COL_LOST)
                banner.set_text("NO TELEMETRY")
                banner.set_color("gray")
            return

        # ---- top-down view
        yaw = state["yaw"]
        car.set_xy(car_polygon(0, 0, yaw))
        car.set_facecolor(col)
        a = math.radians(yaw)
        dx, dy = -math.sin(a), math.cos(a)          # forward direction on screen
        near = s["dist_cm"] < OBSTACLE_CM
        in_range = s["dist_cm"] <= MAX_RANGE_CM
        reach = s["dist_cm"] if in_range else MAX_RANGE_CM
        beam.set_data([dx * 12, dx * reach], [dy * 12, dy * reach])
        beam.set_color("red" if near else "yellow")
        if in_range:
            obst_top.set_data([dx * reach], [dy * reach])
            obst_top.set_color("red" if near else "lime")
        else:
            obst_top.set_data([], [])
        danger.set_alpha(0.3 if near else 0.12)
        valid = bool(s["valid"])
        cl, cr = max(s["cmd_l"], 0), max(s["cmd_r"], 0)
        lbar.set_height(max(cl * 0.9, 0.5))
        rbar.set_height(max(cr * 0.9, 0.5))
        for bar in (lbar, rbar):
            bar.set_facecolor("#3ddc84" if valid else COL_LOST)
        ltxt.set_text(f"L {cl}")
        rtxt.set_text(f"R {cr}")
        msgs = [name] if name != ST_NAMES[0] else []
        if near and name != ST_NAMES[2]:
            msgs.append("OBSTACLE NEAR")
        banner.set_text("  ".join(msgs) if msgs else "clear")
        banner.set_color("lime" if not msgs else ("orange" if not valid or near else "red"))

        c = s["corr"]
        steer = "straight" if c == 0 else (f"left  {c:+d}" if c > 0 else f"right {c:+d}")
        jmin = s["jit_min_us"] if s["jit_min_us"] < 100000 else "--"
        sk_txt = ("--" if s["skid"] < 0 else ("ON" if s["skid"] else "off"))
        status.set_text(
            f"t         {s['t_ms'] / 1000:8.1f} s\n"
            f"yaw rate  {s['gz_dps']:8.1f} deg/s{'' if valid else '  (stale)'}\n"
            f"heading   {yaw:8.1f} deg\n"
            f"IMU valid {'yes' if valid else 'NO':>8}\n"
            f"skid      {sk_txt:>8}   events {s['ev']}\n"
            f"steer     {steer:>8}\n"
            f"jitter    {jmin}..{s['jit_max_us']} us\n"
            f"misses {s['miss']}  i2c err {s['i2c_err']}  resets {s['rst']}"
            + ("" if s["tg"] < 0 else f"\nboard: target {s['tg']}%  accel {s['ac']}  cmds rx {s['rx']}")
        )

        # ---- graphs
        t = np.array(H["t"])
        l_dist.set_data(t, np.array(H["dist"]))
        l_gz.set_data(t, np.array(H["gz"]))
        l_cl.set_data(t, np.array(H["cl"]))
        l_cr.set_data(t, np.array(H["cr"]))
        l_spd.set_data(t, np.array(H["spd"]))
        l_tspd.set_data(t, np.array(H["tspd"]))
        jmin_a = np.array(H["jmin"], dtype=float)
        jmax_a = np.array(H["jmax"], dtype=float)
        l_jmin.set_data(t, np.minimum(jmin_a, JIT_PLOT_MAX_US))
        l_jmax.set_data(t, np.minimum(jmax_a, JIT_PLOT_MAX_US))
        ax1.set_xlim(t[-1] - HISTORY_S, t[-1] + 0.5)
        vals = np.array(H["gz"], dtype=float)
        vals = vals[~np.isnan(vals)]
        peak = float(np.abs(vals).max()) if vals.size else 0.0
        lim = max(peak * 1.2, SKID_DPS * 1.5)
        ax2.set_ylim(-lim, lim)
        top = min(max(2100.0, float(np.nanmax(jmax_a)) + 20), JIT_PLOT_MAX_US)
        lo = 1900.0
        if not np.all(np.isnan(jmin_a)):
            lo = max(0.0, min(1900.0, float(np.nanmin(jmin_a)) - 50))
        ax5.set_ylim(lo, top)
        redraw_shading(t)

    fig.sim = sim                        # handles for testing
    fig.sliders = (sl_speed, sl_acc)
    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    try:
        plt.show()
    finally:
        stop_flag.set()
    return anim


if __name__ == "__main__":
    main()