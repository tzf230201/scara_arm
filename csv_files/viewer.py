#!/usr/bin/env python3
import argparse
import json
import time
import threading
from collections import deque

try:
    import zmq
except ImportError:
    raise SystemExit("pyzmq belum terinstall. Install: pip install pyzmq")

import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed to enable 3D)


# =========================
# Forward kinematics (mengikuti versi kamu)
# =========================
def servo_forward_kinematics(angle_1_deg: float) -> float:
    # z (mm) = (90/360) * angle_1
    return (90.0 / 360.0) * float(angle_1_deg)

def arm_forward_kinematics(angle_2: float, angle_3: float, angle_4: float):
    # NOTE: mengikuti versi kamu sebelumnya
    angle_4 = -float(angle_4)

    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5
    RATIO = 5.0

    theta2 = (float(angle_2) / RATIO) + OFFSET_2
    theta3 = (float(angle_3) / RATIO) + OFFSET_3 - (float(angle_2) / RATIO)

    t2 = math.radians(theta2)
    t3 = math.radians(theta3)

    x2 = L2 * math.cos(t2)
    y2 = L2 * math.sin(t2)
    x3 = x2 + L3 * math.cos(t2 + t3)
    y3 = y2 + L3 * math.sin(t2 + t3)

    yaw = (angle_4 / RATIO) + OFFSET_4
    return x3, y3, yaw


# =========================
# ZMQ parsing
# =========================
def parse_pvt(msg: str):
    try:
        obj = json.loads(msg)
    except json.JSONDecodeError:
        return None
    if not isinstance(obj, dict):
        return None
    if obj.get("command") != "pvt_point":
        return None
    req = ["p1","v1","p2","v2","p3","v3","p4","v4","t_ms"]
    for k in req:
        if k not in obj:
            return None
    return obj


class RateMeter:
    def __init__(self, window=200):
        self.ts = deque(maxlen=window)
    def tick(self, t):
        self.ts.append(t)
    def hz(self):
        if len(self.ts) < 2:
            return 0.0
        dt = self.ts[-1] - self.ts[0]
        if dt <= 0:
            return 0.0
        return (len(self.ts) - 1) / dt


# =========================
# Receiver thread
# =========================
class PVTReceiver:
    def __init__(self, endpoint: str, mode: str, conflate: bool, rate_window: int = 200):
        self.endpoint = endpoint
        self.mode = mode
        self.conflate = conflate

        self.ctx = zmq.Context.instance()
        if mode == "SUB":
            self.sock = self.ctx.socket(zmq.SUB)
            self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        else:
            self.sock = self.ctx.socket(zmq.PULL)

        if conflate:
            # keep only the latest message (viewer anti-lag)
            self.sock.setsockopt(zmq.CONFLATE, 1)

        # viewer biasanya connect
        self.sock.connect(endpoint)

        self._stop = threading.Event()
        self.lock = threading.Lock()

        self.count_pvt = 0
        self.count_bad = 0
        self.rate = RateMeter(window=rate_window)

        self.last = None  # (t_rcv, pvt_dict, x,y,z,yaw)
        self.hist_x = deque(maxlen=5000)
        self.hist_y = deque(maxlen=5000)
        self.hist_z = deque(maxlen=5000)

        self.csv_f = None
        self.log_csv = False

        self.thread = threading.Thread(target=self._run, daemon=True)

    def enable_csv(self, path: str):
        self.csv_f = open(path, "w", encoding="utf-8")
        self.csv_f.write("recv_time_s,t_ms,p1,v1,p2,v2,p3,v3,p4,v4,x,y,z,yaw\n")
        self.csv_f.flush()
        self.log_csv = True

    def start(self):
        self.thread.start()

    def stop(self):
        self._stop.set()
        try:
            self.sock.close(0)
        except Exception:
            pass
        try:
            if self.csv_f:
                self.csv_f.flush()
                self.csv_f.close()
        except Exception:
            pass

    def _run(self):
        poller = zmq.Poller()
        poller.register(self.sock, zmq.POLLIN)

        while not self._stop.is_set():
            events = dict(poller.poll(timeout=200))
            if self.sock not in events:
                continue

            raw = self.sock.recv()
            try:
                msg = raw.decode("utf-8", errors="replace")
            except Exception:
                msg = str(raw)

            pvt = parse_pvt(msg)
            if pvt is None:
                with self.lock:
                    self.count_bad += 1
                continue

            t_rcv = time.time()
            self.rate.tick(t_rcv)

            p1 = float(pvt["p1"])
            p2 = float(pvt["p2"])
            p3 = float(pvt["p3"])
            p4 = float(pvt["p4"])

            z = servo_forward_kinematics(p1)
            x, y, yaw = arm_forward_kinematics(p2, p3, p4)

            with self.lock:
                self.count_pvt += 1
                self.last = (t_rcv, pvt, x, y, z, yaw)
                self.hist_x.append(x)
                self.hist_y.append(y)
                self.hist_z.append(z)

                if self.log_csv and self.csv_f:
                    self.csv_f.write(
                        f"{t_rcv:.6f},{int(pvt['t_ms'])},"
                        f"{float(pvt['p1'])},{float(pvt['v1'])},"
                        f"{float(pvt['p2'])},{float(pvt['v2'])},"
                        f"{float(pvt['p3'])},{float(pvt['v3'])},"
                        f"{float(pvt['p4'])},{float(pvt['v4'])},"
                        f"{x},{y},{z},{yaw}\n"
                    )
                    if (self.count_pvt % 50) == 0:
                        self.csv_f.flush()


def set_3d_limits(ax, xs, ys, zs, pad_min=10.0):
    """Autoscale 3D bounds biar enak dilihat."""
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)

    rx = xmax - xmin
    ry = ymax - ymin
    rz = zmax - zmin
    pad_x = max(pad_min, 0.1 * (rx + 1e-9))
    pad_y = max(pad_min, 0.1 * (ry + 1e-9))
    pad_z = max(pad_min, 0.1 * (rz + 1e-9))

    ax.set_xlim(xmin - pad_x, xmax + pad_x)
    ax.set_ylim(ymin - pad_y, ymax + pad_y)
    ax.set_zlim(zmin - pad_z, zmax + pad_z)


# =========================
# Main + Animation
# =========================
def main():
    ap = argparse.ArgumentParser(description="Real-time ZMQ PVT Viewer (3D XYZ animation)")
    ap.add_argument("--endpoint", default="ipc:///tmp/motor_cmd",
                    help="ZMQ endpoint, default ipc:///tmp/motor_cmd")
    ap.add_argument("--mode", choices=["SUB", "PULL"], default="SUB",
                    help="SUB untuk PUB/SUB, PULL untuk PUSH/PULL")
    ap.add_argument("--conflate", action="store_true",
                    help="ambil message terbaru saja (hindari lag)")
    ap.add_argument("--trail", type=int, default=1000,
                    help="berapa titik trail yang ditampilkan (default 1000)")
    ap.add_argument("--interval", type=int, default=50,
                    help="refresh interval animasi (ms), default 50")
    ap.add_argument("--save", default="",
                    help="opsional: simpan log CSV, contoh --save pvt_log.csv")
    ap.add_argument("--fixed", action="store_true",
                    help="kalau ON, tidak autoscale (pakai limit default).")
    args = ap.parse_args()

    recv = PVTReceiver(args.endpoint, args.mode, args.conflate, rate_window=200)
    if args.save:
        recv.enable_csv(args.save)
        print(f"[Viewer] logging CSV -> {args.save}")

    print(f"[Viewer] connect {args.mode} -> {args.endpoint}")
    print("[Viewer] start (jalankan viewer dulu, baru UI publisher)\n")
    recv.start()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("Waiting for PVT...")

    # initial dummy limits (kalau fixed)
    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    ax.set_zlim(-50, 300)

    line, = ax.plot([], [], [], linewidth=1.5)
    pt, = ax.plot([], [], [], marker="o", linestyle="None")

    def update(_frame):
        with recv.lock:
            last = recv.last
            xs = list(recv.hist_x)[-args.trail:]
            ys = list(recv.hist_y)[-args.trail:]
            zs = list(recv.hist_z)[-args.trail:]
            hz = recv.rate.hz()
            n_ok = recv.count_pvt
            n_bad = recv.count_bad

        if not xs:
            ax.set_title("Waiting for PVT...")
            return line, pt

        line.set_data(xs, ys)
        line.set_3d_properties(zs)
        pt.set_data([xs[-1]], [ys[-1]])
        pt.set_3d_properties([zs[-1]])

        if not args.fixed:
            set_3d_limits(ax, xs, ys, zs, pad_min=10.0)

        if last:
            _, _, x, y, z, yaw = last
            ax.set_title(
                f"XYZ Trajectory | X={x:.1f} Y={y:.1f} Z={z:.1f} Yaw={yaw:.1f} | "
                f"rate≈{hz:.1f}Hz | ok={n_ok} bad={n_bad}"
            )

        return line, pt

    ani = FuncAnimation(fig, update, interval=args.interval, blit=False)

    try:
        plt.show()
    finally:
        recv.stop()


if __name__ == "__main__":
    main()
