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
    "motions_dir": "./Scara_Arm_CSV_files",
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


def save_config(path: str, cfg: dict):
    out = json.loads(json.dumps(cfg))
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
        f.write("\n")


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
    cats = {
        # Future detailed packs
        "park_pickup": [],
        "pickup_indown_outdown": [],
        "pickup_indown_outup": [],
        "pickup_inup_outdown": [],
        "pickup_inup_outup": [],
        "shuttle_pickup": [],
        "turn_pickup": [],
        "park_place": [],
        "place_indown_outdown": [],
        "place_indown_outup": [],
        "place_inup_outdown": [],
        "place_inup_outup": [],
        "shuttle_place": [],
        "turn_place": [],
        "home": [],
        # Backward-compatible buckets used by current UI/runtime
        "pickup": [],
        "place": [],
        "shuttle": [],
        "park": [],
    }
    if not os.path.isdir(motions_dir):
        return cats

    for root, _, files in os.walk(motions_dir):
        root_rel_l = os.path.relpath(root, motions_dir).lower()
        for fn in sorted(files):
            if not fn.lower().endswith(".csv"):
                continue
            p = os.path.join(root, fn)
            n = fn.lower()
            prefix = n.split("-", 1)[0].strip()

            is_pickup_ctx = ("pickup_task" in root_rel_l) or ("pickup" in n)
            is_place_ctx = ("place_task" in root_rel_l) or ("place" in n)
            is_indown = "indown" in n
            is_inup = "inup" in n
            is_outdown = "outdown" in n
            is_outup = "outup" in n
            is_shuttle = ("shuttle" in n) or ("shuttel" in n)  # support typo: shuttel_place
            is_turn = "turn" in n
            # Prefix-driven classification:
            # 00 => park, 01/10 => pickup, 02 => place, 03 => shuttle, 99 => turn
            is_park = ("park" in n) or (prefix == "00")
            is_home = ("home" in n) or ("home" in root_rel_l)

            is_pickup_code = prefix in ("01", "10")
            is_place_code = prefix == "02"
            is_shuttle_code = prefix == "03"
            is_turn_code = prefix == "99"

            # Prefer numbered code mapping; fallback to keyword mapping if code is absent.
            is_pickup_motion = is_pickup_code or ((prefix not in ("00", "01", "02", "03", "10", "99")) and is_pickup_ctx)
            is_place_motion = is_place_code or ((prefix not in ("00", "01", "02", "03", "10", "99")) and is_place_ctx)
            is_shuttle_motion = is_shuttle_code or ((prefix not in ("00", "01", "02", "03", "10", "99")) and is_shuttle)

            # Detailed pickup packs
            if is_pickup_ctx and is_park:
                cats["park_pickup"].append(p)
            if is_pickup_ctx and is_pickup_motion and is_indown and is_outdown:
                cats["pickup_indown_outdown"].append(p)
            if is_pickup_ctx and is_pickup_motion and is_indown and is_outup:
                cats["pickup_indown_outup"].append(p)
            if is_pickup_ctx and is_pickup_motion and is_inup and is_outdown:
                cats["pickup_inup_outdown"].append(p)
            if is_pickup_ctx and is_pickup_motion and is_inup and is_outup:
                cats["pickup_inup_outup"].append(p)
            if is_pickup_ctx and is_shuttle_motion:
                cats["shuttle_pickup"].append(p)
            if is_pickup_ctx and (is_turn or is_turn_code):
                cats["turn_pickup"].append(p)

            # Detailed place packs
            if is_place_ctx and is_park:
                cats["park_place"].append(p)
            if is_place_ctx and is_place_motion and is_indown and is_outdown:
                cats["place_indown_outdown"].append(p)
            if is_place_ctx and is_place_motion and is_indown and is_outup:
                cats["place_indown_outup"].append(p)
            if is_place_ctx and is_place_motion and is_inup and is_outdown:
                cats["place_inup_outdown"].append(p)
            if is_place_ctx and is_place_motion and is_inup and is_outup:
                cats["place_inup_outup"].append(p)
            if is_place_ctx and is_shuttle_motion:
                cats["shuttle_place"].append(p)
            if is_place_ctx and (is_turn or is_turn_code):
                cats["turn_place"].append(p)

            # Home pack
            if is_home:
                cats["home"].append(p)

            # Backward-compatible buckets
            if is_pickup_ctx and (is_pickup_motion or is_park):
                cats["pickup"].append(p)
            if is_place_ctx and (is_place_motion or is_park):
                cats["place"].append(p)
            if is_shuttle_motion:
                cats["shuttle"].append(p)
            if is_park or is_home:
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
class HoverTooltip:
    def __init__(self, widget, text: str, delay_ms: int = 300):
        self.widget = widget
        self.text = text
        self.delay_ms = delay_ms
        self.tip_win = None
        self._after_id = None
        self.widget.bind("<Enter>", self._on_enter, add="+")
        self.widget.bind("<Leave>", self._on_leave, add="+")
        self.widget.bind("<ButtonPress>", self._on_leave, add="+")

    def _on_enter(self, _event=None):
        self._cancel()
        self._after_id = self.widget.after(self.delay_ms, self._show)

    def _on_leave(self, _event=None):
        self._cancel()
        self._hide()

    def _cancel(self):
        if self._after_id is not None:
            try:
                self.widget.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None

    def _show(self):
        if self.tip_win is not None:
            return
        x = self.widget.winfo_rootx() + 18
        y = self.widget.winfo_rooty() + 18
        self.tip_win = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        lbl = tk.Label(
            tw,
            text=self.text,
            justify="left",
            relief="solid",
            borderwidth=1,
            background="#ffffe0",
            padx=6,
            pady=4,
            wraplength=380
        )
        lbl.pack()

    def _hide(self):
        if self.tip_win is not None:
            try:
                self.tip_win.destroy()
            except Exception:
                pass
            self.tip_win = None


class SettingsDialog(tk.Toplevel):
    def __init__(self, parent: "TaskSchedulerGUI"):
        super().__init__(parent)
        self.parent = parent
        self.title("Settings")
        self.geometry("760x680")
        self.transient(parent)
        self.grab_set()

        self.config_path_var = tk.StringVar(value=parent.config_path)

        self.motion_dir_var = tk.StringVar()

        self.pickup_z_base_var = tk.StringVar()
        self.pickup_z_step_var = tk.StringVar()
        self.place_z_base_var = tk.StringVar()
        self.place_z_step_var = tk.StringVar()
        self.shuttle_z_base_var = tk.StringVar()
        self.shuttle_z_step_var = tk.StringVar()
        self.park_z_base_var = tk.StringVar()
        self.park_z_step_var = tk.StringVar()

        self.dt_ms_var = tk.StringVar()

        self.step_mm_var = tk.StringVar()
        self.angle_threshold_var = tk.StringVar()
        self.acc_dec_ms_var = tk.StringVar()
        self.v_max_var = tk.StringVar()
        self.v_safe_var = tk.StringVar()
        self.offset_z_global_var = tk.StringVar()

        self.pre_enable_var = tk.BooleanVar(value=True)
        self.pre_yaw_radius_var = tk.StringVar()
        self.pre_min_dist_var = tk.StringVar()

        self.zmq_endpoint_var = tk.StringVar()
        self.zmq_mode_var = tk.StringVar()
        self.zmq_dry_run_var = tk.BooleanVar(value=False)
        self._tooltips: List[HoverTooltip] = []
        self._csv_combos: Dict[str, ttk.Combobox] = {}
        self._csv_vars: Dict[str, tk.StringVar] = {}

        # Share CSV selections with main GUI (single source of truth)
        self.pickup_csv_var = parent.pickup_csv_var
        self.place_csv_var = parent.place_csv_var
        self.shuttle_csv_var = parent.shuttle_csv_var
        self.park_csv_var = parent.park_csv_var

        # Read-only display vars for detailed CSV categories
        for k in [
            "park_pickup",
            "pickup_indown_outdown",
            "pickup_indown_outup",
            "pickup_inup_outdown",
            "pickup_inup_outup",
            "shuttle_pickup",
            "turn_pickup",
            "park_place",
            "place_indown_outdown",
            "place_indown_outup",
            "place_inup_outdown",
            "place_inup_outup",
            "shuttle_place",
            "turn_place",
            "home",
        ]:
            self._csv_vars[k] = tk.StringVar()

        self._build_ui()
        self._fill_from_cfg(parent.cfg)
        self.refresh_csv_options()

    def _add_hint(self, parent, row: int, col: int, text: str):
        bubble = tk.Canvas(parent, width=16, height=16, highlightthickness=0, cursor="question_arrow")
        bubble.grid(row=row, column=col, sticky="w", padx=(4, 6))
        bubble.create_oval(1, 1, 15, 15, outline="#666666", fill="#f6f6f6")
        bubble.create_text(8, 8, text="?", fill="#333333", font=("TkDefaultFont", 8, "bold"))
        tip = HoverTooltip(bubble, text=text)
        self._tooltips.append(tip)

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        top = ttk.Frame(self, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(1, weight=1)
        ttk.Label(top, text="Config file:").grid(row=0, column=0, sticky="w")
        ttk.Entry(top, textvariable=self.config_path_var).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(top, text="Load", command=self._on_load).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(top, text="Save", command=self._on_save).grid(row=0, column=3)

        body_wrap = ttk.Frame(self, padding=(10, 0, 10, 10))
        body_wrap.grid(row=1, column=0, sticky="nsew")
        body_wrap.columnconfigure(0, weight=1)
        body_wrap.rowconfigure(0, weight=1)

        canvas = tk.Canvas(body_wrap, highlightthickness=0)
        canvas.grid(row=0, column=0, sticky="nsew")
        vsb = ttk.Scrollbar(body_wrap, orient="vertical", command=canvas.yview)
        vsb.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=vsb.set)

        content = ttk.Frame(canvas)
        win = canvas.create_window((0, 0), window=content, anchor="nw")
        content.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(win, width=e.width))

        content.columnconfigure(0, weight=1)

        row = 0
        row = self._section_general(content, row)
        row = self._section_csv_selection(content, row)
        row = self._section_z_by_task(content, row)
        row = self._section_pvt(content, row)
        row = self._section_profile(content, row)
        row = self._section_pre_motion(content, row)
        self._section_zmq(content, row)

        bot = ttk.Frame(self, padding=10)
        bot.grid(row=2, column=0, sticky="ew")
        bot.columnconfigure(0, weight=1)
        ttk.Button(bot, text="Apply", command=self._on_apply).grid(row=0, column=1, sticky="e")
        ttk.Button(bot, text="Close", command=self.destroy).grid(row=0, column=2, sticky="e", padx=(8, 0))

    def _section_general(self, parent, row_start: int) -> int:
        sec = ttk.LabelFrame(parent, text="General", padding=10)
        sec.grid(row=row_start, column=0, sticky="ew")
        sec.columnconfigure(2, weight=1)
        ttk.Label(sec, text="motions_dir").grid(row=0, column=0, sticky="w")
        self._add_hint(sec, 0, 1, "Main folder used to scan motion CSV files (Pickup/Place/Home, etc.).")
        ttk.Entry(sec, textvariable=self.motion_dir_var).grid(row=0, column=2, sticky="ew", padx=6)
        ttk.Button(sec, text="Browse", command=self._browse_motions_dir).grid(row=0, column=3)
        return row_start + 1

    def _section_csv_selection(self, parent, row_start: int) -> int:
        sec = ttk.LabelFrame(parent, text="CSV Selection", padding=10)
        sec.grid(row=row_start, column=0, sticky="ew", pady=(8, 0))
        sec.columnconfigure(1, weight=1)

        rows = [
            ("Pickup CSV", "pickup", self.pickup_csv_var, "Select Pickup CSV", True),
            ("Place CSV", "place", self.place_csv_var, "Select Place CSV", True),
            ("Shuttle CSV", "shuttle", self.shuttle_csv_var, "Select Shuttle CSV", True),
            ("Home/Park CSV", "home_park", self.park_csv_var, "Select Home/Park CSV", True),

            ("park_pickup", "park_pickup", self._csv_vars["park_pickup"], "", False),
            ("pickup_indown_outdown", "pickup_indown_outdown", self._csv_vars["pickup_indown_outdown"], "", False),
            ("pickup_indown_outup", "pickup_indown_outup", self._csv_vars["pickup_indown_outup"], "", False),
            ("pickup_inup_outdown", "pickup_inup_outdown", self._csv_vars["pickup_inup_outdown"], "", False),
            ("pickup_inup_outup", "pickup_inup_outup", self._csv_vars["pickup_inup_outup"], "", False),
            ("shuttle_pickup", "shuttle_pickup", self._csv_vars["shuttle_pickup"], "", False),
            ("turn_pickup", "turn_pickup", self._csv_vars["turn_pickup"], "", False),
            ("park_place", "park_place", self._csv_vars["park_place"], "", False),
            ("place_indown_outdown", "place_indown_outdown", self._csv_vars["place_indown_outdown"], "", False),
            ("place_indown_outup", "place_indown_outup", self._csv_vars["place_indown_outup"], "", False),
            ("place_inup_outdown", "place_inup_outdown", self._csv_vars["place_inup_outdown"], "", False),
            ("place_inup_outup", "place_inup_outup", self._csv_vars["place_inup_outup"], "", False),
            ("shuttle_place", "shuttle_place", self._csv_vars["shuttle_place"], "", False),
            ("turn_place", "turn_place", self._csv_vars["turn_place"], "", False),
            ("home", "home", self._csv_vars["home"], "", False),
        ]
        for r, (label, key, var, title, can_browse) in enumerate(rows):
            ttk.Label(sec, text=label).grid(row=r, column=0, sticky="w", pady=(0, 6) if r < len(rows)-1 else (0, 0))
            cb = ttk.Combobox(sec, textvariable=var, state="readonly")
            cb.grid(row=r, column=1, sticky="ew", padx=6, pady=(0, 6) if r < len(rows)-1 else (0, 0))
            if can_browse:
                ttk.Button(sec, text="Browse", command=lambda v=var, t=title: self._browse_csv(v, t)).grid(
                    row=r, column=2, pady=(0, 6) if r < len(rows)-1 else (0, 0)
                )
            self._csv_combos[key] = cb

        actions = ttk.Frame(sec)
        actions.grid(row=len(rows), column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Button(actions, text="Scan motions", command=self._scan_csv_now).grid(row=0, column=0, sticky="w")
        self._add_hint(actions, 0, 1, "Scan motion folder and refresh CSV lists for pickup/place/shuttle/home-park.")
        return row_start + 1

    def _section_z_by_task(self, parent, row_start: int) -> int:
        sec = ttk.LabelFrame(parent, text="Z By Task", padding=10)
        sec.grid(row=row_start, column=0, sticky="ew", pady=(8, 0))
        labels = ["pickup", "place", "shuttle", "park"]
        vars_base = [self.pickup_z_base_var, self.place_z_base_var, self.shuttle_z_base_var, self.park_z_base_var]
        vars_step = [self.pickup_z_step_var, self.place_z_step_var, self.shuttle_z_step_var, self.park_z_step_var]

        ttk.Label(sec, text="task").grid(row=0, column=0, sticky="w")
        ttk.Label(sec, text="z_base_mm").grid(row=0, column=1, sticky="w")
        self._add_hint(sec, 0, 2, "Base Z offset (mm) for the selected task.")
        ttk.Label(sec, text="z_step_mm").grid(row=0, column=3, sticky="w")
        self._add_hint(sec, 0, 4, "Additional Z offset per level: z_base + (level - 1) * z_step.")
        for i, name in enumerate(labels, start=1):
            ttk.Label(sec, text=name).grid(row=i, column=0, sticky="w")
            ttk.Entry(sec, textvariable=vars_base[i - 1], width=16).grid(row=i, column=1, sticky="w", padx=6)
            ttk.Entry(sec, textvariable=vars_step[i - 1], width=16).grid(row=i, column=3, sticky="w", padx=6)
        return row_start + 1

    def _section_pvt(self, parent, row_start: int) -> int:
        sec = ttk.LabelFrame(parent, text="PVT", padding=10)
        sec.grid(row=row_start, column=0, sticky="ew", pady=(8, 0))
        ttk.Label(sec, text="dt_ms").grid(row=0, column=0, sticky="w")
        self._add_hint(sec, 0, 1, "Time period between PVT points (milliseconds). Must be > 0.")
        ttk.Entry(sec, textvariable=self.dt_ms_var, width=16).grid(row=0, column=2, sticky="w", padx=6)
        return row_start + 1

    def _section_profile(self, parent, row_start: int) -> int:
        sec = ttk.LabelFrame(parent, text="Profile", padding=10)
        sec.grid(row=row_start, column=0, sticky="ew", pady=(8, 0))
        fields = [
            ("step_mm", self.step_mm_var, "Sampling distance used to detect direction changes along the path (mm)."),
            ("angle_threshold", self.angle_threshold_var, "Angle threshold (degrees) used to flag sharp/critical points."),
            ("acc_dec_ms", self.acc_dec_ms_var, "Acceleration and deceleration time for the trapezoid profile (ms)."),
            ("v_max", self.v_max_var, "Maximum path speed (mm/s)."),
            ("v_safe", self.v_safe_var, "Safe speed used on critical segments (mm/s)."),
            ("offset_z_global", self.offset_z_global_var, "Global Z offset added to all segments (mm)."),
        ]
        for i, (label, var, hint) in enumerate(fields):
            ttk.Label(sec, text=label).grid(row=i, column=0, sticky="w")
            self._add_hint(sec, i, 1, hint)
            ttk.Entry(sec, textvariable=var, width=16).grid(row=i, column=2, sticky="w", padx=6)
        return row_start + 1

    def _section_pre_motion(self, parent, row_start: int) -> int:
        sec = ttk.LabelFrame(parent, text="Straight Pre Motion", padding=10)
        sec.grid(row=row_start, column=0, sticky="ew", pady=(8, 0))
        ttk.Checkbutton(sec, text="enable", variable=self.pre_enable_var).grid(row=0, column=0, sticky="w")
        self._add_hint(
            sec, 0, 1,
            "Enable a straight pre-motion before following the CSV path to avoid abrupt pose jumps. "
            "Without pre-motion, the robot may 'jump' to the start of the trajectory (sudden position/yaw changes)."
        )
        ttk.Label(sec, text="yaw_arc_radius_mm").grid(row=1, column=0, sticky="w")
        self._add_hint(
            sec, 1, 1,
            "Equivalent radius (mm) used to convert yaw change into arc distance. "
            "This is the weighting factor that turns yaw change into an 'equivalent distance'."
        )
        ttk.Entry(sec, textvariable=self.pre_yaw_radius_var, width=16).grid(row=1, column=2, sticky="w", padx=6)
        ttk.Label(sec, text="min_distance_mm").grid(row=2, column=0, sticky="w")
        self._add_hint(sec, 2, 1, "If distance to the CSV start point is below this threshold, pre-motion is skipped.")
        ttk.Entry(sec, textvariable=self.pre_min_dist_var, width=16).grid(row=2, column=2, sticky="w", padx=6)
        return row_start + 1

    def _section_zmq(self, parent, row_start: int):
        sec = ttk.LabelFrame(parent, text="ZMQ", padding=10)
        sec.grid(row=row_start, column=0, sticky="ew", pady=(8, 0))
        sec.columnconfigure(2, weight=1)
        ttk.Label(sec, text="endpoint").grid(row=0, column=0, sticky="w")
        self._add_hint(sec, 0, 1, "Target ZMQ socket address for sending PVT commands, e.g. ipc:///tmp/motor_cmd")
        ttk.Entry(sec, textvariable=self.zmq_endpoint_var).grid(row=0, column=2, sticky="ew", padx=6)
        ttk.Label(sec, text="mode").grid(row=1, column=0, sticky="w")
        self._add_hint(sec, 1, 1, "PUB_BIND: app binds. PUB_CONNECT/PUSH_CONNECT: app connects to the endpoint.")
        ttk.Combobox(
            sec,
            textvariable=self.zmq_mode_var,
            state="readonly",
            values=["PUB_BIND", "PUB_CONNECT", "PUSH_CONNECT"],
            width=16,
        ).grid(row=1, column=2, sticky="w", padx=6)
        ttk.Checkbutton(sec, text="dry_run", variable=self.zmq_dry_run_var).grid(row=2, column=0, sticky="w")
        self._add_hint(sec, 2, 1, "If enabled, commands are not actually sent to ZMQ (simulation/log only).")

    def _browse_motions_dir(self):
        path = filedialog.askdirectory(
            title="Select motions directory",
            initialdir=self.motion_dir_var.get() or os.path.dirname(self.parent.config_path),
            parent=self
        )
        if path:
            self.motion_dir_var.set(path)

    def _browse_csv(self, var: tk.StringVar, title: str):
        md = self.parent.cfg["motions_dir"]
        path = filedialog.askopenfilename(
            title=title,
            initialdir=md,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            parent=self
        )
        if path:
            var.set(os.path.abspath(path))

    def _scan_csv_now(self):
        self.parent.scan_and_set_defaults()
        self.refresh_csv_options()

    def refresh_csv_options(self):
        if not self._csv_combos:
            return

        home_or_park = self.parent.scan_results["home"] if self.parent.scan_results["home"] else self.parent.scan_results["park"]

        mapping = {
            "pickup": self.parent.scan_results["pickup"],
            "place": self.parent.scan_results["place"],
            "shuttle": self.parent.scan_results["shuttle"],
            "home_park": home_or_park,
            "park_pickup": self.parent.scan_results["park_pickup"],
            "pickup_indown_outdown": self.parent.scan_results["pickup_indown_outdown"],
            "pickup_indown_outup": self.parent.scan_results["pickup_indown_outup"],
            "pickup_inup_outdown": self.parent.scan_results["pickup_inup_outdown"],
            "pickup_inup_outup": self.parent.scan_results["pickup_inup_outup"],
            "shuttle_pickup": self.parent.scan_results["shuttle_pickup"],
            "turn_pickup": self.parent.scan_results["turn_pickup"],
            "park_place": self.parent.scan_results["park_place"],
            "place_indown_outdown": self.parent.scan_results["place_indown_outdown"],
            "place_indown_outup": self.parent.scan_results["place_indown_outup"],
            "place_inup_outdown": self.parent.scan_results["place_inup_outdown"],
            "place_inup_outup": self.parent.scan_results["place_inup_outup"],
            "shuttle_place": self.parent.scan_results["shuttle_place"],
            "turn_place": self.parent.scan_results["turn_place"],
            "home": self.parent.scan_results["home"],
        }
        for key, values in mapping.items():
            cb = self._csv_combos.get(key)
            if cb is not None:
                cb["values"] = values
                var = cb.cget("textvariable")
                if isinstance(var, str) and var:
                    cur = self.getvar(var)
                    if cur not in values:
                        self.setvar(var, values[0] if values else "")

    def _f(self, v: tk.StringVar, name: str) -> float:
        try:
            return float(v.get())
        except Exception as e:
            raise ValueError(f"Invalid float for '{name}': {v.get()}") from e

    def _i(self, v: tk.StringVar, name: str) -> int:
        try:
            return int(v.get())
        except Exception as e:
            raise ValueError(f"Invalid int for '{name}': {v.get()}") from e

    def _to_cfg(self) -> dict:
        cfg = json.loads(json.dumps(DEFAULT_CONFIG))
        cfg["motions_dir"] = self.motion_dir_var.get().strip()

        cfg["z_by_task"]["pickup"]["z_base_mm"] = self._f(self.pickup_z_base_var, "z_by_task.pickup.z_base_mm")
        cfg["z_by_task"]["pickup"]["z_step_mm"] = self._f(self.pickup_z_step_var, "z_by_task.pickup.z_step_mm")
        cfg["z_by_task"]["place"]["z_base_mm"] = self._f(self.place_z_base_var, "z_by_task.place.z_base_mm")
        cfg["z_by_task"]["place"]["z_step_mm"] = self._f(self.place_z_step_var, "z_by_task.place.z_step_mm")
        cfg["z_by_task"]["shuttle"]["z_base_mm"] = self._f(self.shuttle_z_base_var, "z_by_task.shuttle.z_base_mm")
        cfg["z_by_task"]["shuttle"]["z_step_mm"] = self._f(self.shuttle_z_step_var, "z_by_task.shuttle.z_step_mm")
        cfg["z_by_task"]["park"]["z_base_mm"] = self._f(self.park_z_base_var, "z_by_task.park.z_base_mm")
        cfg["z_by_task"]["park"]["z_step_mm"] = self._f(self.park_z_step_var, "z_by_task.park.z_step_mm")

        cfg["pvt"]["dt_ms"] = self._i(self.dt_ms_var, "pvt.dt_ms")

        cfg["profile"]["step_mm"] = self._f(self.step_mm_var, "profile.step_mm")
        cfg["profile"]["angle_threshold"] = self._f(self.angle_threshold_var, "profile.angle_threshold")
        cfg["profile"]["acc_dec_ms"] = self._f(self.acc_dec_ms_var, "profile.acc_dec_ms")
        cfg["profile"]["v_max"] = self._f(self.v_max_var, "profile.v_max")
        cfg["profile"]["v_safe"] = self._f(self.v_safe_var, "profile.v_safe")
        cfg["profile"]["offset_z_global"] = self._f(self.offset_z_global_var, "profile.offset_z_global")

        cfg["straight_pre_motion"]["enable"] = bool(self.pre_enable_var.get())
        cfg["straight_pre_motion"]["yaw_arc_radius_mm"] = self._f(self.pre_yaw_radius_var, "straight_pre_motion.yaw_arc_radius_mm")
        cfg["straight_pre_motion"]["min_distance_mm"] = self._f(self.pre_min_dist_var, "straight_pre_motion.min_distance_mm")

        cfg["zmq"]["endpoint"] = self.zmq_endpoint_var.get().strip()
        cfg["zmq"]["mode"] = self.zmq_mode_var.get().strip()
        cfg["zmq"]["dry_run"] = bool(self.zmq_dry_run_var.get())

        path = self.config_path_var.get().strip() or self.parent.config_path
        return load_config_from_inline(cfg, path)

    def _fill_from_cfg(self, cfg: dict):
        self.motion_dir_var.set(str(cfg["motions_dir"]))

        z = cfg["z_by_task"]
        self.pickup_z_base_var.set(str(z["pickup"]["z_base_mm"]))
        self.pickup_z_step_var.set(str(z["pickup"]["z_step_mm"]))
        self.place_z_base_var.set(str(z["place"]["z_base_mm"]))
        self.place_z_step_var.set(str(z["place"]["z_step_mm"]))
        self.shuttle_z_base_var.set(str(z["shuttle"]["z_base_mm"]))
        self.shuttle_z_step_var.set(str(z["shuttle"]["z_step_mm"]))
        self.park_z_base_var.set(str(z["park"]["z_base_mm"]))
        self.park_z_step_var.set(str(z["park"]["z_step_mm"]))

        self.dt_ms_var.set(str(cfg["pvt"]["dt_ms"]))

        p = cfg["profile"]
        self.step_mm_var.set(str(p["step_mm"]))
        self.angle_threshold_var.set(str(p["angle_threshold"]))
        self.acc_dec_ms_var.set(str(p["acc_dec_ms"]))
        self.v_max_var.set(str(p["v_max"]))
        self.v_safe_var.set(str(p["v_safe"]))
        self.offset_z_global_var.set(str(p.get("offset_z_global", 0.0)))

        spm = cfg["straight_pre_motion"]
        self.pre_enable_var.set(bool(spm["enable"]))
        self.pre_yaw_radius_var.set(str(spm["yaw_arc_radius_mm"]))
        self.pre_min_dist_var.set(str(spm["min_distance_mm"]))

        zq = cfg["zmq"]
        self.zmq_endpoint_var.set(str(zq["endpoint"]))
        self.zmq_mode_var.set(str(zq["mode"]))
        self.zmq_dry_run_var.set(bool(zq.get("dry_run", False)))

    def _on_apply(self):
        try:
            cfg = self._to_cfg()
            cfg_path = os.path.abspath(self.config_path_var.get().strip() or self.parent.config_path)
            self.parent.apply_settings(cfg, cfg_path)
            self.refresh_csv_options()
            self.parent._log("[CFG] Applied from Settings.")
        except Exception as e:
            messagebox.showerror("Settings Error", str(e), parent=self)

    def _on_load(self):
        path = filedialog.askopenfilename(
            title="Load config JSON",
            initialdir=os.path.dirname(self.config_path_var.get() or self.parent.config_path),
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            parent=self
        )
        if not path:
            return
        try:
            cfg = load_config(path)
            self.config_path_var.set(os.path.abspath(path))
            self._fill_from_cfg(cfg)
            self.parent.apply_settings(cfg, os.path.abspath(path))
            self.refresh_csv_options()
            self.parent._log(f"[CFG] Loaded: {os.path.basename(path)}")
        except Exception as e:
            messagebox.showerror("Load Error", str(e), parent=self)

    def _on_save(self):
        try:
            cfg = self._to_cfg()
        except Exception as e:
            messagebox.showerror("Settings Error", str(e), parent=self)
            return

        current = self.config_path_var.get().strip() or self.parent.config_path
        path = filedialog.asksaveasfilename(
            title="Save config JSON",
            initialdir=os.path.dirname(current),
            initialfile=os.path.basename(current),
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            parent=self
        )
        if not path:
            return
        try:
            save_config(path, cfg)
            self.config_path_var.set(os.path.abspath(path))
            self.parent.apply_settings(cfg, os.path.abspath(path))
            self.refresh_csv_options()
            self.parent._log(f"[CFG] Saved: {os.path.basename(path)}")
        except Exception as e:
            messagebox.showerror("Save Error", str(e), parent=self)


def load_config_from_inline(base_cfg: dict, config_path: str) -> dict:
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    deep_update(cfg, base_cfg)

    config_dir = os.path.dirname(os.path.abspath(config_path))
    if not os.path.isabs(cfg["motions_dir"]):
        cfg["motions_dir"] = os.path.normpath(os.path.join(config_dir, cfg["motions_dir"]))

    cfg["pvt"]["dt_ms"] = int(cfg["pvt"]["dt_ms"])
    if cfg["pvt"]["dt_ms"] <= 0:
        raise ValueError("pvt.dt_ms must be > 0")

    mode = cfg["zmq"]["mode"]
    if mode not in ("PUB_BIND", "PUB_CONNECT", "PUSH_CONNECT"):
        raise ValueError("zmq.mode must be PUB_BIND / PUB_CONNECT / PUSH_CONNECT")

    return cfg


class TaskSchedulerGUI(tk.Tk):
    def __init__(self, config_path: str):
        super().__init__()
        self.title("TASK GUI")
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

        self.scan_results = {
            "park_pickup": [],
            "pickup_indown_outdown": [],
            "pickup_indown_outup": [],
            "pickup_inup_outdown": [],
            "pickup_inup_outup": [],
            "shuttle_pickup": [],
            "turn_pickup": [],
            "park_place": [],
            "place_indown_outdown": [],
            "place_indown_outup": [],
            "place_inup_outdown": [],
            "place_inup_outup": [],
            "shuttle_place": [],
            "turn_place": [],
            "home": [],
            "pickup": [],
            "place": [],
            "shuttle": [],
            "park": [],
        }
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
        ttk.Button(top, text="Settings", command=self.open_settings).grid(row=0, column=0, sticky="w")

        main = ttk.Frame(self, padding=10)
        main.grid(row=1, column=0, sticky="nsew")
        # Keep Task/Queue/Log proportions stable so controls stay inside their panels.
        main.columnconfigure(0, weight=2, minsize=360)
        main.columnconfigure(1, weight=1, minsize=220)
        main.columnconfigure(2, weight=2, minsize=420)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.columnconfigure(0, weight=1)

        tasks = ttk.LabelFrame(left, text="Task Selection", padding=10)
        tasks.grid(row=0, column=0, sticky="ew")

        self._build_task_selector(tasks, "FIRST TASK", self.first_kind_var, self.first_level_var).grid(row=0, column=0, sticky="ew", pady=(0,10))
        self._build_task_selector(tasks, "SECOND TASK", self.second_kind_var, self.second_level_var).grid(row=1, column=0, sticky="ew")

        btns = ttk.Frame(tasks)
        btns.grid(row=2, column=0, sticky="ew", pady=(10,0))
        btns.columnconfigure(0, weight=1)
        btns.columnconfigure(1, weight=1)
        ttk.Button(btns, text="Enqueue", command=self.enqueue_pair, padding=(8, 8)).grid(
            row=0, column=0, sticky="nsew", padx=(0,6)
        )
        ttk.Button(btns, text="Clear Queue", command=self.clear_queue, padding=(8, 8)).grid(
            row=0, column=1, sticky="nsew"
        )

        runbtns = ttk.Frame(tasks)
        runbtns.grid(row=3, column=0, sticky="ew", pady=(10,0))
        runbtns.columnconfigure(0, weight=1)
        runbtns.columnconfigure(1, weight=1)
        ttk.Button(runbtns, text="START", command=self.start_run).grid(row=0, column=0, sticky="ew", padx=(0,6))
        ttk.Button(runbtns, text="STOP", command=self.stop_run).grid(row=0, column=1, sticky="ew")

        # ✅ NEW checkbox UI
        tk.Checkbutton(
            tasks,
            text="Keep ready (stay running when queue empty)",
            variable=self.keep_ready_var,
            anchor="w",
            justify="left",
            wraplength=300
        ).grid(row=4, column=0, sticky="ew", pady=(10,0))

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

    def open_settings(self):
        SettingsDialog(self)

    def apply_settings(self, cfg: dict, config_path: str):
        self.cfg = cfg
        self.config_path = os.path.abspath(config_path)
        if self.worker:
            self.worker.apply_cfg(self.cfg)
        self.scan_and_set_defaults()

    def scan_and_set_defaults(self):
        md = self.cfg["motions_dir"]
        self.scan_results = scan_motions(md)

        def set_values(values: List[str], var: tk.StringVar):
            if var.get() in values:
                return
            var.set(values[0] if values else "")

        set_values(self.scan_results["pickup"], self.pickup_csv_var)
        set_values(self.scan_results["place"], self.place_csv_var)
        set_values(self.scan_results["shuttle"], self.shuttle_csv_var)
        home_or_park = self.scan_results["home"] if self.scan_results["home"] else self.scan_results["park"]
        set_values(home_or_park, self.park_csv_var)

        self._log(
            f"[SCAN] pickup={len(self.scan_results['pickup'])}, "
            f"place={len(self.scan_results['place'])}, "
            f"home={len(self.scan_results['home'])}, "
            f"shuttle={len(self.scan_results['shuttle'])}, "
            f"park={len(self.scan_results['park'])}"
        )
        self._log(
            "[SCAN:DETAIL] "
            f"park_pickup={len(self.scan_results['park_pickup'])}, "
            f"pickup_indown_outdown={len(self.scan_results['pickup_indown_outdown'])}, "
            f"pickup_indown_outup={len(self.scan_results['pickup_indown_outup'])}, "
            f"pickup_inup_outdown={len(self.scan_results['pickup_inup_outdown'])}, "
            f"pickup_inup_outup={len(self.scan_results['pickup_inup_outup'])}, "
            f"shuttle_pickup={len(self.scan_results['shuttle_pickup'])}, "
            f"turn_pickup={len(self.scan_results['turn_pickup'])}, "
            f"park_place={len(self.scan_results['park_place'])}, "
            f"place_indown_outdown={len(self.scan_results['place_indown_outdown'])}, "
            f"place_indown_outup={len(self.scan_results['place_indown_outup'])}, "
            f"place_inup_outdown={len(self.scan_results['place_inup_outdown'])}, "
            f"place_inup_outup={len(self.scan_results['place_inup_outup'])}, "
            f"shuttle_place={len(self.scan_results['shuttle_place'])}, "
            f"turn_place={len(self.scan_results['turn_place'])}"
        )
        self._refresh_queue()

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
