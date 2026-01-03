#!/usr/bin/env python3
import os
import json
import time
import math
import queue
import threading
from dataclasses import dataclass
from typing import Optional, List, Callable, Dict, Tuple

import numpy as np
import pandas as pd
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import zmq
    ZMQ_AVAILABLE = True
except Exception:
    ZMQ_AVAILABLE = False


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_CONFIG = {
    "motions_dir": "./motions",
    "z_by_task": {
        "pickup":  {"z_base_mm": 0.0, "z_step_mm": 24.0},
        "place":   {"z_base_mm": 0.0, "z_step_mm": 24.0},
        "shuttle": {"z_base_mm": 0.0, "z_step_mm": 0.0},
        "park":    {"z_base_mm": 0.0, "z_step_mm": 0.0},
    },
    "pvt": {"dt_ms": 50},
    "profile": {
        "step_mm": 2.0,
        "angle_threshold": 5.0,
        "acc_dec_ms": 2000.0,
        "v_max": 100.0,
        "v_safe": 10.0,
        "offset_z_global": 0.0
    },
    "straight_pre_motion": {
        "enable": True,
        "yaw_arc_radius_mm": 258.0,
        "min_distance_mm": 0.2
    },
    "zmq": {
        "endpoint": "ipc:///tmp/motor_cmd",
        "mode": "PUB_BIND",     # PUB_BIND / PUB_CONNECT / PUSH_CONNECT
        "dry_run": False
    }
}


def deep_update(dst: dict, src: dict) -> dict:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_update(dst[k], v)
        else:
            dst[k] = v
    return dst


def load_config(path: str) -> dict:
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    config_dir = SCRIPT_DIR

    if os.path.exists(path):
        config_dir = os.path.dirname(os.path.abspath(path))
        with open(path, "r", encoding="utf-8") as f:
            txt = f.read()
        if txt.strip():  # handle empty file
            user_cfg = json.loads(txt)
            deep_update(cfg, user_cfg)

    if not os.path.isabs(cfg["motions_dir"]):
        cfg["motions_dir"] = os.path.normpath(os.path.join(config_dir, cfg["motions_dir"]))

    cfg["pvt"]["dt_ms"] = int(cfg["pvt"]["dt_ms"])
    if cfg["pvt"]["dt_ms"] <= 0:
        raise ValueError("pvt.dt_ms must be > 0")

    mode = cfg["zmq"]["mode"]
    if mode not in ("PUB_BIND", "PUB_CONNECT", "PUSH_CONNECT"):
        raise ValueError("zmq.mode must be PUB_BIND / PUB_CONNECT / PUSH_CONNECT")

    return cfg


# =========================
# Kinematics (IK + FK)
# =========================
def servo_check_limit(angle_1):
    if angle_1 > (3510 * 4):
        angle_1 = (3510 * 4)
    elif angle_1 < -2:
        angle_1 = -2
    return angle_1

def servo_inverse_kinematics(z_mm: float) -> float:
    angle_1 = (360.0 / 90.0) * float(z_mm)
    return float(servo_check_limit(angle_1))

def servo_forward_kinematics(angle_1_deg: float) -> float:
    return (90.0 / 360.0) * float(angle_1_deg)

def arm_check_limit(angle_2, angle_3, angle_4):
    angle_2_upper_limit = 178 * 5
    angle_2_lower_limit = 0
    angle_3_upper_limit = 0 + angle_2
    angle_3_lower_limit = (-135 * 5) + angle_2
    angle_4_upper_limit = (196 * 5) + angle_3
    angle_4_lower_limit = 0 + angle_3
    angle_4 *= -1

    angle_2 = min(max(angle_2, angle_2_lower_limit), angle_2_upper_limit)
    angle_3 = min(max(angle_3, angle_3_lower_limit), angle_3_upper_limit)
    angle_4 = min(max(angle_4, angle_4_lower_limit), angle_4_upper_limit)

    angle_4 *= -1
    return float(angle_2), float(angle_3), float(angle_4)

def arm_inverse_kinematics(x, y, yaw):
    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134
    OFFSET_4 = -52.5
    RATIO = 5.0

    distance = math.hypot(x, y)
    max_reach = L2 + L3
    if distance > max_reach:
        scale = max_reach / (distance + 1e-9)
        x *= scale
        y *= scale

    cos_theta3 = (x**2 + y**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))
    theta3_rad = math.acos(cos_theta3)
    theta3 = math.degrees(theta3_rad)

    k1 = L2 + L3 * cos_theta3
    k2 = L3 * math.sin(theta3_rad)
    theta2_rad = math.atan2(y, x) - math.atan2(k2, k1)
    theta2 = math.degrees(theta2_rad)

    joint_2 = (theta2 - OFFSET_2) * RATIO
    joint_3 = (theta3 - OFFSET_3) * RATIO + joint_2
    joint_4 = -((yaw - OFFSET_4) * RATIO)

    return arm_check_limit(joint_2, joint_3, joint_4)

def arm_forward_kinematics(angle_2: float, angle_3: float, angle_4: float):
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
    return float(x3), float(y3), float(yaw)


# =========================
# Z offset helper
# =========================
def z_offset_for(cfg: dict, kind: str, level: Optional[int]) -> float:
    zmap = cfg.get("z_by_task", {})
    k = kind.lower().strip()
    base = float(zmap.get(k, {}).get("z_base_mm", 0.0))
    step = float(zmap.get(k, {}).get("z_step_mm", 0.0))
    if level is None:
        return base
    return base + (int(level) - 1) * step


# =========================
# Scan motions folder
# =========================
def scan_motions(motions_dir: str) -> Dict[str, List[str]]:
    cats = {"pickup": [], "place": [], "shuttle": [], "park": []}
    if not os.path.isdir(motions_dir):
        return cats

    for root, _, files in os.walk(motions_dir):
        for fn in sorted(files):
            if not fn.lower().endswith(".csv"):
                continue
            p = os.path.join(root, fn)
            n = fn.lower()

            if n.startswith("pickup"):
                cats["pickup"].append(p)
            elif n.startswith("place"):
                cats["place"].append(p)
            elif n.startswith("shuttle"):
                cats["shuttle"].append(p)

            if "park" in n:
                cats["park"].append(p)

    for k in cats:
        seen = set()
        out = []
        for p in cats[k]:
            if p not in seen:
                out.append(p)
                seen.add(p)
        cats[k] = out
    return cats


# =========================
# CSV -> PVT arrays (XYZC only) + offset_z
# =========================
def robot_csv_to_pvt_arrays(
    input_csv: str,
    step_mm: float,
    angle_threshold: float,
    vmax: float,
    vsafe: float,
    t_acc_ms: float,
    dt_ms: int,
    offset_z: float = 0.0,
):
    df = pd.read_csv(input_csv)
    for col in ["X", "Y", "Z", "C"]:
        if col not in df.columns:
            raise ValueError(f"CSV missing '{col}' (required: X,Y,Z,C): {input_csv}")

    pts = df[["X", "Y", "Z"]].values.astype(float)
    dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum_dist = np.insert(np.cumsum(dists), 0, 0.0)

    n = len(pts)
    flags = np.zeros(n, dtype=int)

    for i in range(n):
        l = np.searchsorted(cum_dist, cum_dist[i] - step_mm, side="right") - 1
        r = np.searchsorted(cum_dist, cum_dist[i] + step_mm, side="left")
        if 0 <= l < i and r > i and r < n:
            v1 = pts[i] - pts[l]
            v2 = pts[r] - pts[i]
            n1 = np.linalg.norm(v1)
            n2 = np.linalg.norm(v2)
            if n1 > 1e-9 and n2 > 1e-9:
                cosv = float(np.dot(v1, v2) / (n1 * n2))
                ang = math.degrees(math.acos(max(-1.0, min(1.0, cosv))))
                flags[i] = 1 if (ang > angle_threshold) else 0

    df["dangerous"] = flags.astype(int)
    points = df[["X", "Y", "Z", "C", "dangerous"]].values.astype(float)

    def compute_path_length(points_xyz):
        seg_lengths = np.sqrt(np.sum(np.diff(points_xyz, axis=0) ** 2, axis=1))
        return seg_lengths, np.insert(np.cumsum(seg_lengths), 0, 0.0)

    def trapezoid_profile(D, v_start, v_end, vmax_, t_acc_ms_, dt_ms_):
        dt = dt_ms_ / 1000.0
        t_acc = t_acc_ms_ / 1000.0
        t_dec = t_acc
        acc = (vmax_ - v_start) / t_acc if t_acc > 0 else 1e9
        dec = (vmax_ - v_end) / t_dec if t_dec > 0 else 1e9

        s_acc = (vmax_**2 - v_start**2) / (2 * acc) if acc > 0 else 0
        s_dec = (vmax_**2 - v_end**2) / (2 * dec) if dec > 0 else 0

        if s_acc + s_dec > D:
            vmax_ = math.sqrt(max(0.0, (2 * D * acc * dec + dec * v_start**2 + acc * v_end**2) / (acc + dec)))
            s_acc = (vmax_**2 - v_start**2) / (2 * acc)
            s_dec = (vmax_**2 - v_end**2) / (2 * dec)
            s_cruise = 0.0
        else:
            s_cruise = D - (s_acc + s_dec)

        t_acc = (vmax_ - v_start) / acc if acc > 0 else 0.0
        t_dec = (vmax_ - v_end) / dec if dec > 0 else 0.0
        t_cruise = s_cruise / vmax_ if vmax_ > 0 else 0.0
        T = t_acc + t_cruise + t_dec

        times = np.arange(0, T + dt, dt)
        v_vals = []
        for t in times:
            if t < t_acc:
                v = v_start + acc * t
            elif t < t_acc + t_cruise:
                v = vmax_
            elif t <= T:
                v = vmax_ - dec * (t - (t_acc + t_cruise))
            else:
                v = v_end
            v_vals.append(max(float(v), 0.0))
        return times, np.array(v_vals, dtype=float)

    def build_s_profile(points_xyzc, seg_lengths, vmax_, vsafe_, t_acc_ms_, dt_ms_):
        s_all = []
        offset_s, v_cur, i = 0.0, 0.0, 0

        while i < len(seg_lengths):
            safe_len = 0.0
            while i < len(seg_lengths) and points_xyzc[i, 4] == 0 and points_xyzc[i + 1, 4] == 0:
                safe_len += float(seg_lengths[i])
                i += 1
            if safe_len > 0:
                v_end = vsafe_ if i < len(seg_lengths) else 0.0
                _, v = trapezoid_profile(safe_len, v_cur, v_end, vmax_, t_acc_ms_, dt_ms_)
                s = np.cumsum(v) * (dt_ms_ / 1000.0)
                s_all.extend(list(offset_s + s))
                v_cur = v_end
                offset_s = float(s_all[-1])

            danger_len = 0.0
            while i < len(seg_lengths) and (points_xyzc[i, 4] == 1 or points_xyzc[i + 1, 4] == 1):
                danger_len += float(seg_lengths[i])
                i += 1
            if danger_len > 0:
                dt = dt_ms_ / 1000.0
                steps = int(danger_len / max(vsafe_ * dt, 1e-9)) + 1
                v = np.ones(steps, dtype=float) * vsafe_
                s = np.cumsum(v) * dt
                s_all.extend(list(offset_s + s))
                v_cur = vsafe_
                offset_s = float(s_all[-1])

        return np.array(s_all, dtype=float)

    def interpolate_position(points_xyz, cum_lengths, s):
        idx = int(np.searchsorted(cum_lengths, s) - 1)
        idx = max(0, min(idx, len(points_xyz) - 2))
        s0, s1 = float(cum_lengths[idx]), float(cum_lengths[idx + 1])
        r = (s - s0) / (s1 - s0 + 1e-9)
        pos = (1.0 - r) * points_xyz[idx, :3] + r * points_xyz[idx + 1, :3]
        return pos, idx

    seg_lengths, cum_lengths = compute_path_length(points[:, :3])
    s_vals = build_s_profile(points, seg_lengths, vmax, vsafe, t_acc_ms, dt_ms)

    dt_s = dt_ms / 1000.0
    p1, p2, p3, p4 = [], [], [], []
    for s in s_vals:
        pos, idx = interpolate_position(points[:, :3], cum_lengths, float(s))
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2] + offset_z)
        c_val = float(points[idx, 3])

        p1.append(float(servo_inverse_kinematics(z)))
        a2, a3, a4 = arm_inverse_kinematics(x, y, c_val)
        p2.append(float(a2)); p3.append(float(a3)); p4.append(float(a4))

    def vel(arr):
        if len(arr) < 2:
            return [0.0] * len(arr)
        out = [(arr[i + 1] - arr[i]) / dt_s for i in range(len(arr) - 1)]
        out.append(0.0)
        return out

    v1, v2, v3, v4 = vel(p1), vel(p2), vel(p3), vel(p4)
    return p1, v1, p2, v2, p3, v3, p4, v4


# =========================
# ZMQ Sender (REUSED)
# =========================
class CommandSender:
    def __init__(self, cfg: dict, log_fn: Callable[[str], None]):
        self.log = log_fn
        self.endpoint = cfg["zmq"]["endpoint"]
        self.mode = cfg["zmq"]["mode"]
        self.dry_run = bool(cfg["zmq"].get("dry_run", False))

        self.sock = None
        self.ctx = None

        if self.dry_run:
            self.log("[Sender] Dry-run ON (no ZMQ).")
            return

        if not ZMQ_AVAILABLE:
            raise RuntimeError("pyzmq not available. Install: pip install pyzmq")

        self.ctx = zmq.Context.instance()

        if self.mode == "PUB_BIND":
            self.sock = self.ctx.socket(zmq.PUB)
            self.sock.bind(self.endpoint)
            self.log(f"[Sender] PUB bind -> {self.endpoint}")
            time.sleep(0.1)
        elif self.mode == "PUB_CONNECT":
            self.sock = self.ctx.socket(zmq.PUB)
            self.sock.connect(self.endpoint)
            self.log(f"[Sender] PUB connect -> {self.endpoint}")
            time.sleep(0.1)
        elif self.mode == "PUSH_CONNECT":
            self.sock = self.ctx.socket(zmq.PUSH)
            self.sock.connect(self.endpoint)
            self.log(f"[Sender] PUSH connect -> {self.endpoint}")
        else:
            raise ValueError(f"Unknown zmq.mode: {self.mode}")

        self.sock.linger = 0

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close(0)
            except Exception:
                pass
            self.sock = None

    def signature(self) -> Tuple[str, str, bool]:
        return (self.endpoint, self.mode, self.dry_run)

    def send_json(self, obj: dict):
        if self.dry_run:
            return
        if self.sock is None:
            return
        self.sock.send_string(json.dumps(obj))

    def send_pvt_point(self, p1,v1,p2,v2,p3,v3,p4,v4,t_ms: int):
        self.send_json({
            "command": "pvt_point",
            "p1": float(p1), "v1": float(v1),
            "p2": float(p2), "v2": float(v2),
            "p3": float(p3), "v3": float(v3),
            "p4": float(p4), "v4": float(v4),
            "t_ms": int(t_ms),
        })


# =========================
# Tasks and steps
# =========================
@dataclass
class Task:
    kind: str
    level: Optional[int] = None
    def label(self) -> str:
        if self.kind in ("Pickup", "Place"):
            return f"{self.kind} L{int(self.level):02d}"
        return "Shuttle"

@dataclass
class CsvSegment:
    path: str
    offset_z: float = 0.0

@dataclass
class WorkerStep:
    name: str
    segment: CsvSegment
    skip_if: Optional[Callable[[], bool]] = None
    on_complete: Optional[Callable[[], None]] = None


def task_to_segment(cfg: dict, task: Task, pickup_csv: str, place_csv: str, shuttle_csv: str) -> CsvSegment:
    if task.kind == "Pickup":
        return CsvSegment(pickup_csv, offset_z=z_offset_for(cfg, "pickup", int(task.level)))
    if task.kind == "Place":
        return CsvSegment(place_csv, offset_z=z_offset_for(cfg, "place", int(task.level)))
    return CsvSegment(shuttle_csv, offset_z=z_offset_for(cfg, "shuttle", None))

def park_segment(cfg: dict, park_csv: str) -> Optional[CsvSegment]:
    if not park_csv or (not os.path.exists(park_csv)):
        return None
    return CsvSegment(park_csv, offset_z=z_offset_for(cfg, "park", None))


# =========================
# Worker (reused), with pre-straight motion
# =========================
class PVTMotionWorker:
    def __init__(self, sender: CommandSender, cfg: dict, log_fn: Callable[[str], None]):
        self.sender = sender
        self.log = log_fn
        self._steps = queue.Queue()
        self._stop_evt = threading.Event()
        self._active_lock = threading.Lock()
        self._active = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._started = False

        # start from HOME joints=0
        self.last_joints = [0.0, 0.0, 0.0, 0.0]
        x, y, yaw = arm_forward_kinematics(0.0, 0.0, 0.0)
        z = servo_forward_kinematics(0.0)
        self.last_pose = [x, y, z, yaw]

        self.apply_cfg(cfg)

    def apply_cfg(self, cfg: dict):
        self.dt_ms = int(cfg["pvt"]["dt_ms"])
        self.step_mm = float(cfg["profile"]["step_mm"])
        self.angle_threshold = float(cfg["profile"]["angle_threshold"])
        self.vmax = float(cfg["profile"]["v_max"])
        self.vsafe = float(cfg["profile"]["v_safe"])
        self.acc_dec_ms = float(cfg["profile"]["acc_dec_ms"])
        self.offset_z_global = float(cfg["profile"].get("offset_z_global", 0.0))

        spm = cfg.get("straight_pre_motion", {})
        self.enable_pre = bool(spm.get("enable", True))
        self.yaw_arc_radius = float(spm.get("yaw_arc_radius_mm", 258.0))
        self.min_dist = float(spm.get("min_distance_mm", 0.2))

        self.realtime_sleep = True

    def start(self):
        if not self._started:
            self._started = True
            self._thread.start()

    def enqueue_step(self, step: WorkerStep):
        self._steps.put(step)

    def clear_steps(self):
        try:
            while True:
                self._steps.get_nowait()
        except queue.Empty:
            pass

    def stop(self):
        self._stop_evt.set()

    def clear_stop(self):
        self._stop_evt.clear()

    def is_idle(self) -> bool:
        with self._active_lock:
            active = self._active
        return (not active) and self._steps.empty()

    def _set_active(self, v: bool):
        with self._active_lock:
            self._active = v

    def _send_joint_points(self, joints_list: List[List[float]]):
        dt_s = self.dt_ms / 1000.0
        for i in range(len(joints_list)):
            if self._stop_evt.is_set():
                return
            cur = np.array(joints_list[i], dtype=float)
            if i < len(joints_list) - 1:
                nxt = np.array(joints_list[i + 1], dtype=float)
                v = (nxt - cur) / dt_s
            else:
                v = np.zeros(4, dtype=float)

            self.sender.send_pvt_point(
                cur[0], v[0],
                cur[1], v[1],
                cur[2], v[2],
                cur[3], v[3],
                self.dt_ms
            )
            if self.realtime_sleep:
                time.sleep(dt_s)

    def _pre_straight_to_csv_start(self, csv_path: str, offset_z_abs: float):
        if not self.enable_pre:
            return

        df0 = pd.read_csv(csv_path)
        for col in ["X", "Y", "Z", "C"]:
            if col not in df0.columns:
                raise ValueError(f"CSV missing '{col}' (required X,Y,Z,C): {csv_path}")

        x_t = float(df0.iloc[0]["X"])
        y_t = float(df0.iloc[0]["Y"])
        z_t = float(df0.iloc[0]["Z"]) + float(offset_z_abs)
        yaw_t = float(df0.iloc[0]["C"])

        x0, y0, z0, yaw0 = self.last_pose
        dx, dy, dz = (x_t - x0), (y_t - y0), (z_t - z0)
        dyaw = (yaw_t - yaw0)

        arc = float(self.yaw_arc_radius) * abs(math.radians(dyaw))
        D = math.sqrt(dx*dx + dy*dy + dz*dz + arc*arc)

        if D < self.min_dist:
            return

        speed = max(self.vmax, 1e-6)
        T_s = max(D / speed, self.dt_ms / 1000.0)
        dt_s = self.dt_ms / 1000.0
        steps = max(1, int(math.ceil(T_s / dt_s)))

        self.log(f"[PRE] Straight motion | D≈{D:.2f}mm | T≈{T_s:.3f}s | v_max={self.vmax:.1f}mm/s")

        joints_list = []
        for i in range(1, steps + 1):
            if self._stop_evt.is_set():
                return
            r = i / steps
            x = x0 + r * dx
            y = y0 + r * dy
            z = z0 + r * dz
            yaw = yaw0 + r * dyaw

            p1 = servo_inverse_kinematics(z)
            p2, p3, p4 = arm_inverse_kinematics(x, y, yaw)
            joints_list.append([float(p1), float(p2), float(p3), float(p4)])

        self._send_joint_points(joints_list)

        if joints_list:
            self.last_joints = joints_list[-1]
            x, y, yaw = arm_forward_kinematics(self.last_joints[1], self.last_joints[2], self.last_joints[3])
            z = servo_forward_kinematics(self.last_joints[0])
            self.last_pose = [x, y, z, yaw]

    def _run(self):
        while True:
            step: WorkerStep = self._steps.get()

            if step.skip_if and step.skip_if():
                continue

            self._set_active(True)
            try:
                self._execute_step(step)
            finally:
                self._set_active(False)
                if self._stop_evt.is_set():
                    self._stop_evt.clear()

            if step.on_complete:
                try:
                    step.on_complete()
                except Exception:
                    pass

    def _execute_step(self, step: WorkerStep):
        seg = step.segment
        path = seg.path

        if not os.path.exists(path):
            self.log(f"[WARN] Missing CSV: {path}")
            return

        offset_z_abs = self.offset_z_global + float(seg.offset_z)

        self.log(f"\n=== STEP: {step.name} ===")
        self.log(f"[CSV] {os.path.basename(path)} (offset_z={offset_z_abs:.3f})")

        self._pre_straight_to_csv_start(path, offset_z_abs)
        if self._stop_evt.is_set():
            return

        p1,v1,p2,v2,p3,v3,p4,v4 = robot_csv_to_pvt_arrays(
            input_csv=path,
            step_mm=self.step_mm,
            angle_threshold=self.angle_threshold,
            vmax=self.vmax,
            vsafe=self.vsafe,
            t_acc_ms=self.acc_dec_ms,
            dt_ms=self.dt_ms,
            offset_z=offset_z_abs
        )

        N = min(len(p1), len(p2), len(p3), len(p4))
        dt_s = self.dt_ms / 1000.0
        for i in range(N):
            if self._stop_evt.is_set():
                return
            self.sender.send_pvt_point(
                p1[i], v1[i],
                p2[i], v2[i],
                p3[i], v3[i],
                p4[i], v4[i],
                self.dt_ms
            )
            if self.realtime_sleep:
                time.sleep(dt_s)

        if N > 0:
            self.last_joints = [float(p1[N-1]), float(p2[N-1]), float(p3[N-1]), float(p4[N-1])]
            x, y, yaw = arm_forward_kinematics(self.last_joints[1], self.last_joints[2], self.last_joints[3])
            z = servo_forward_kinematics(self.last_joints[0])
            self.last_pose = [x, y, z, yaw]


# =========================
# GUI
# =========================
class TaskSchedulerGUI(tk.Tk):
    def __init__(self, config_path: str):
        super().__init__()
        self.title("Task GUI (Pickup/Place/Shuttle + Level Z Offset, Auto Park, Anti-Teleport)")
        self.geometry("1150x740")

        self.config_path = os.path.abspath(config_path)
        self.cfg = load_config(self.config_path)

        self._queue_lock = threading.Lock()
        self.task_queue: List[Task] = []

        self.running = False
        self.at_park = False

        self._log_q = queue.Queue()

        self.sender: Optional[CommandSender] = None
        self.sender_sig: Optional[Tuple[str, str, bool]] = None
        self.worker: Optional[PVTMotionWorker] = None

        self.scan_results = {"pickup": [], "place": [], "shuttle": [], "park": []}
        self.pickup_csv_var = tk.StringVar()
        self.place_csv_var = tk.StringVar()
        self.shuttle_csv_var = tk.StringVar()
        self.park_csv_var = tk.StringVar()

        self.first_kind_var = tk.StringVar(value="Pickup")
        self.first_level_var = tk.IntVar(value=1)
        self.second_kind_var = tk.StringVar(value="Place")
        self.second_level_var = tk.IntVar(value=1)

        # ✅ NEW: keep-ready checkbox
        self.keep_ready_var = tk.BooleanVar(value=True)

        self._build_ui()
        self.scan_and_set_defaults()

        self.after(100, self._flush_log)
        self.after(200, self._scheduler_tick)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        try:
            if self.worker:
                self.worker.stop()
                self.worker.clear_steps()
        except Exception:
            pass
        try:
            if self.sender:
                self.sender.close()
        except Exception:
            pass
        self.destroy()

    def get_queue_len(self) -> int:
        with self._queue_lock:
            return len(self.task_queue)

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        top = ttk.Frame(self, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        ttk.Button(top, text="Reload Config", command=self.reload_config).grid(row=0, column=0, sticky="w")
        ttk.Button(top, text="Scan motions", command=self.scan_and_set_defaults).grid(row=0, column=1, sticky="w", padx=(8,0))

        main = ttk.Frame(self, padding=10)
        main.grid(row=1, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.columnconfigure(2, weight=2)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.columnconfigure(0, weight=1)

        sel = ttk.LabelFrame(left, text="CSV Selection (auto scan)", padding=10)
        sel.grid(row=0, column=0, sticky="ew")
        sel.columnconfigure(1, weight=1)

        self.pickup_cb = self._row_combo(sel, 0, "Pickup CSV:", self.pickup_csv_var, self.browse_pickup)
        self.place_cb  = self._row_combo(sel, 1, "Place CSV:",  self.place_csv_var,  self.browse_place, pady=(6,0))
        self.shut_cb   = self._row_combo(sel, 2, "Shuttle CSV:", self.shuttle_csv_var, self.browse_shuttle, pady=(6,0))
        self.park_cb   = self._row_combo(sel, 3, "Park CSV:",   self.park_csv_var,   self.browse_park, pady=(6,0))

        tasks = ttk.LabelFrame(left, text="Task Selection", padding=10)
        tasks.grid(row=1, column=0, sticky="ew", pady=(10,0))

        self._build_task_selector(tasks, "FIRST TASK", self.first_kind_var, self.first_level_var).grid(row=0, column=0, sticky="ew", pady=(0,10))
        self._build_task_selector(tasks, "SECOND TASK", self.second_kind_var, self.second_level_var).grid(row=1, column=0, sticky="ew")

        btns = ttk.Frame(tasks)
        btns.grid(row=2, column=0, sticky="ew", pady=(10,0))
        btns.columnconfigure(0, weight=1)
        btns.columnconfigure(1, weight=1)
        ttk.Button(btns, text="Enqueue FIRST+SECOND", command=self.enqueue_pair).grid(row=0, column=0, sticky="ew", padx=(0,6))
        ttk.Button(btns, text="Clear Queue", command=self.clear_queue).grid(row=0, column=1, sticky="ew")

        runbtns = ttk.Frame(tasks)
        runbtns.grid(row=3, column=0, sticky="ew", pady=(10,0))
        runbtns.columnconfigure(0, weight=1)
        runbtns.columnconfigure(1, weight=1)
        ttk.Button(runbtns, text="START", command=self.start_run).grid(row=0, column=0, sticky="ew", padx=(0,6))
        ttk.Button(runbtns, text="STOP", command=self.stop_run).grid(row=0, column=1, sticky="ew")

        # ✅ NEW checkbox UI
        ttk.Checkbutton(
            tasks,
            text="Keep ready (stay running when queue empty)",
            variable=self.keep_ready_var
        ).grid(row=4, column=0, sticky="w", pady=(10,0))

        center = ttk.LabelFrame(main, text="Queue", padding=10)
        center.grid(row=0, column=1, sticky="nsew", padx=8)
        center.rowconfigure(0, weight=1)
        center.columnconfigure(0, weight=1)
        self.queue_list = tk.Listbox(center, height=18)
        self.queue_list.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(center, orient="vertical", command=self.queue_list.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.queue_list.configure(yscrollcommand=sb.set)

        right = ttk.LabelFrame(main, text="Log", padding=10)
        right.grid(row=0, column=2, sticky="nsew", padx=(8,0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)

        self.log = tk.Text(right, height=24, wrap="word")
        self.log.grid(row=0, column=0, sticky="nsew")
        log_sb = ttk.Scrollbar(right, orient="vertical", command=self.log.yview)
        log_sb.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=log_sb.set)

    def _row_combo(self, parent, r, label, var, browse_cmd, pady=(0,0)):
        ttk.Label(parent, text=label).grid(row=r, column=0, sticky="w", pady=pady)
        cb = ttk.Combobox(parent, textvariable=var, state="readonly")
        cb.grid(row=r, column=1, sticky="ew", padx=6, pady=pady)
        ttk.Button(parent, text="Browse", command=browse_cmd).grid(row=r, column=2, pady=pady)
        return cb

    def _build_task_selector(self, parent, title, kind_var, level_var) -> ttk.Frame:
        f = ttk.LabelFrame(parent, text=title, padding=10)

        row0 = ttk.Frame(f)
        row0.grid(row=0, column=0, sticky="ew")
        ttk.Radiobutton(row0, text="Pickup", value="Pickup", variable=kind_var,
                        command=lambda: self._update_level_state(f, kind_var)).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(row0, text="Place", value="Place", variable=kind_var,
                        command=lambda: self._update_level_state(f, kind_var)).grid(row=0, column=1, sticky="w")
        ttk.Radiobutton(row0, text="Shuttle", value="Shuttle", variable=kind_var,
                        command=lambda: self._update_level_state(f, kind_var)).grid(row=0, column=2, sticky="w")

        row1 = ttk.Frame(f)
        row1.grid(row=1, column=0, sticky="ew", pady=(8,0))
        ttk.Label(row1, text="Level (1–33):").grid(row=0, column=0, sticky="w")
        sp = ttk.Spinbox(row1, from_=1, to=33, textvariable=level_var, width=8)
        sp.grid(row=0, column=1, sticky="w")
        f._level_spin = sp  # type: ignore

        self._update_level_state(f, kind_var)
        return f

    def _update_level_state(self, frame, kind_var):
        sp = getattr(frame, "_level_spin", None)
        if sp is None:
            return
        sp.configure(state="normal" if kind_var.get() in ("Pickup", "Place") else "disabled")

    def _log(self, s: str):
        self._log_q.put(s)

    def _flush_log(self):
        try:
            while True:
                s = self._log_q.get_nowait()
                self.log.insert("end", s + "\n")
                self.log.see("end")
        except queue.Empty:
            pass
        self.after(100, self._flush_log)

    def reload_config(self):
        try:
            self.cfg = load_config(self.config_path)
            self.scan_and_set_defaults()
            self._log("[CFG] Reloaded.")
        except Exception as e:
            messagebox.showerror("Config Error", str(e))

    def scan_and_set_defaults(self):
        md = self.cfg["motions_dir"]
        self.scan_results = scan_motions(md)

        def set_values(cb: ttk.Combobox, values: List[str], var: tk.StringVar):
            cb["values"] = values
            if var.get() in values:
                return
            var.set(values[0] if values else "")

        set_values(self.pickup_cb, self.scan_results["pickup"], self.pickup_csv_var)
        set_values(self.place_cb,  self.scan_results["place"],  self.place_csv_var)
        set_values(self.shut_cb,   self.scan_results["shuttle"], self.shuttle_csv_var)
        set_values(self.park_cb,   self.scan_results["park"],   self.park_csv_var)

        self._log(f"[SCAN] pickup={len(self.scan_results['pickup'])}, place={len(self.scan_results['place'])}, shuttle={len(self.scan_results['shuttle'])}, park={len(self.scan_results['park'])}")
        self._refresh_queue()

    def browse_pickup(self):  self._browse_any("Select Pickup CSV", self.pickup_csv_var)
    def browse_place(self):   self._browse_any("Select Place CSV", self.place_csv_var)
    def browse_shuttle(self): self._browse_any("Select Shuttle CSV", self.shuttle_csv_var)
    def browse_park(self):    self._browse_any("Select Park CSV", self.park_csv_var)

    def _browse_any(self, title: str, var: tk.StringVar):
        md = self.cfg["motions_dir"]
        path = filedialog.askopenfilename(
            title=title,
            initialdir=md,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if path:
            var.set(os.path.abspath(path))

    def _validate_for_run(self) -> bool:
        def ok(p: str) -> bool:
            return bool(p) and os.path.exists(p)

        if not ok(self.pickup_csv_var.get()):
            messagebox.showerror("Missing", "Pickup CSV hasn't selected yet/ file does not exist.")
            return False
        if not ok(self.place_csv_var.get()):
            messagebox.showerror("Missing", "Place CSV hasn't selected yet/ file does not exist.")
            return False
        if not ok(self.shuttle_csv_var.get()):
            messagebox.showerror("Missing", "Shuttle CSV hasn't selected yet/ file does not exist.")
            return False
        return True

    def enqueue_pair(self):
        t1 = Task(self.first_kind_var.get(), self.first_level_var.get() if self.first_kind_var.get() != "Shuttle" else None)
        t2 = Task(self.second_kind_var.get(), self.second_level_var.get() if self.second_kind_var.get() != "Shuttle" else None)
        with self._queue_lock:
            self.task_queue.extend([t1, t2])
        self._log(f"[UI] Enqueued: {t1.label()} -> {t2.label()}")
        self._refresh_queue()

    def clear_queue(self):
        with self._queue_lock:
            self.task_queue.clear()
        self._log("[UI] Queue cleared.")
        self._refresh_queue()

    def start_run(self):
        if self.running:
            return
        if self.get_queue_len() == 0:
            messagebox.showinfo("Info", "Queue still empty.")
            return
        if not self._validate_for_run():
            return

        try:
            self.cfg = load_config(self.config_path)
        except Exception as e:
            messagebox.showerror("Config Error", str(e))
            return

        try:
            desired_sig = (self.cfg["zmq"]["endpoint"], self.cfg["zmq"]["mode"], bool(self.cfg["zmq"].get("dry_run", False)))
            if (self.sender is None) or (self.sender_sig != desired_sig):
                if self.sender is not None:
                    try:
                        self.sender.close()
                    except Exception:
                        pass
                self.sender = CommandSender(self.cfg, self._log)
                self.sender_sig = desired_sig
        except Exception as e:
            messagebox.showerror("Sender Error", str(e))
            return

        if self.worker is None:
            self.worker = PVTMotionWorker(self.sender, self.cfg, self._log)
            self.worker.start()
        else:
            self.worker.apply_cfg(self.cfg)
            self.worker.clear_stop()

        self.running = True
        self.at_park = False
        self._log("[RUN] START (reused sender/worker)")
        self._refresh_queue()

    def stop_run(self):
        self.running = False
        if self.worker:
            self.worker.stop()
            self.worker.clear_steps()
        self._log("[RUN] STOP")
        self._refresh_queue()

    def _refresh_queue(self):
        self.queue_list.delete(0, tk.END)
        with self._queue_lock:
            for i, t in enumerate(self.task_queue):
                self.queue_list.insert(tk.END, f"{i+1:02d}. {t.label()}")

    def _scheduler_tick(self):
        try:
            if self.running and self.worker and self.worker.is_idle():
                if self.get_queue_len() > 0:
                    self._schedule_next_task()
                else:
                    self._maybe_auto_park_or_stop()
        finally:
            self.after(200, self._scheduler_tick)

    def _schedule_next_task(self):
        with self._queue_lock:
            if not self.task_queue:
                return
            t = self.task_queue.pop(0)

        self._refresh_queue()

        pickup_csv = self.pickup_csv_var.get()
        place_csv  = self.place_csv_var.get()
        shuttle_csv = self.shuttle_csv_var.get()

        seg = task_to_segment(self.cfg, t, pickup_csv, place_csv, shuttle_csv)

        def done():
            self.at_park = False
            self._log(f"[DONE] {t.label()}")

        self.worker.enqueue_step(WorkerStep(
            name=f"Run {t.label()}",
            segment=seg,
            on_complete=lambda: self.after(0, done)
        ))
        self._log(f"[PLAN] {t.label()}")

    def _maybe_auto_park_or_stop(self):
        park_csv = self.park_csv_var.get()
        seg = park_segment(self.cfg, park_csv)

        if (seg is not None) and (not self.at_park):
            def done():
                self.at_park = True
                self._log("[DONE] PARK")

            # kalau ada task baru masuk, park di-skip
            skip_if = lambda: (self.get_queue_len() > 0)

            self.worker.enqueue_step(WorkerStep(
                name="Auto PARK",
                segment=seg,
                skip_if=skip_if,
                on_complete=lambda: self.after(0, done)
            ))
            self._log("[PLAN] PARK")
            return

        # ✅ NEW behavior: keep-ready mode
        if self.keep_ready_var.get():
            # tetap running, standby
            self._log("[RUN] READY (queue empty, waiting new tasks)")
            return

        # default: stop
        self.running = False
        self._log("[RUN] IDLE (queue empty)")
        self._refresh_queue()


def main():
    config_path = os.getenv("TASK_GUI_CONFIG", os.path.join(SCRIPT_DIR, "config.json"))
    app = TaskSchedulerGUI(config_path=config_path)
    app.mainloop()

if __name__ == "__main__":
    main()
