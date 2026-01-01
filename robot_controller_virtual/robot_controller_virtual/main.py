import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger

from sensor_msgs.msg import JointState
from scara_arm_msgs.msg import TesterUICommand

from virtual_motor_msgs.msg import PositionCommand


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def _maybe_rad_to_deg_for_axis(axis_index: int, value: float) -> float:
    # axis 0 is prismatic (meters). Axes 1..3 are revolute (radians <-> degrees).
    if axis_index == 0:
        return float(value)
    return _rad_to_deg(float(value))


def _maybe_deg_to_rad_for_axis(axis_index: int, value: float) -> float:
    if axis_index == 0:
        return float(value)
    return _deg_to_rad(float(value))


def _axis_raw_to_gear_units(axis_index: int, raw_value: float) -> float:
    """Convert internal JointState raw units to motor-node gear units.

    - axis 0: meters (pass-through)
    - axes 1..3: radians -> degrees
    """
    if axis_index == 0:
        return float(raw_value)
    return _rad_to_deg(float(raw_value))


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _smoothstep(u: float) -> float:
    # 3u^2 - 2u^3
    return u * u * (3.0 - 2.0 * u)


def _smoothstep_derivative(u: float) -> float:
    # d/du (3u^2 - 2u^3) = 6u - 6u^2
    return 6.0 * u - 6.0 * u * u


@dataclass
class _Trajectory:
    start: list[float]
    goal: list[float]
    t0: float
    duration: float
    mode: str  # 'pp' | 'pvt'


class VirtualRobotController(Node):
    def __init__(self) -> None:
        super().__init__('robot_controller_virtual')

        self.declare_parameter('cmd_topic', '/scara_arm/tester_ui')
        self.declare_parameter('joint_states_raw_topic', '/joint_states_raw')
        self.declare_parameter('publish_rate_hz', 50.0)

        self.declare_parameter('joint1_name', 'joint_1')
        self.declare_parameter('joint2_name', 'joint_2')
        self.declare_parameter('joint3_name', 'joint_3')
        self.declare_parameter('joint4_name', 'joint_4')

        # multi_virtual_motor interface (motor1..motor4 by default)
        self.declare_parameter('motor_prefixes', ['motor1', 'motor2', 'motor3', 'motor4'])

        # joint_1 in URDF is prismatic. The UI labels it "Joint 1 (deg)", but for this robot
        # we treat Joint 1 input as millimeters to match the rail travel.
        self.declare_parameter('joint1_input_unit', 'mm')  # mm | m

        # Real robot model params (match robot_controller/arm.py)
        self.declare_parameter('gear_ratio', 5.0)
        self.declare_parameter('offset_2_deg', -96.5)
        self.declare_parameter('offset_3_deg', 134.0)
        self.declare_parameter('offset_4_deg', -52.5)
        self.declare_parameter('link2_mm', 137.0)
        self.declare_parameter('link3_mm', 121.0)
        self.declare_parameter('elbow', 'down')  # down | up

        # How to interpret GUI joint degree fields (joints[1..3]):
        # - 'physical': GUI provides physical/gear angles (deg) directly (recommended for RViz + virtual motor)
        # - 'motor': GUI provides motor-side degrees (deg) using robot_controller conventions (ratio+offset mapping)
        # - 'motor_simple': GUI provides motor-side degrees (deg) and we simply divide by gear_ratio (no offsets)
        self.declare_parameter('ui_joint_angle_space', 'physical')  # physical | motor | motor_simple

        self._cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self._js_topic = self.get_parameter('joint_states_raw_topic').get_parameter_value().string_value
        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._j1 = self.get_parameter('joint1_name').get_parameter_value().string_value
        self._j2 = self.get_parameter('joint2_name').get_parameter_value().string_value
        self._j3 = self.get_parameter('joint3_name').get_parameter_value().string_value
        self._j4 = self.get_parameter('joint4_name').get_parameter_value().string_value

        self._j1_unit = self.get_parameter('joint1_input_unit').get_parameter_value().string_value.strip().lower()

        self._gear_ratio = float(self.get_parameter('gear_ratio').value)
        self._off2_deg = float(self.get_parameter('offset_2_deg').value)
        self._off3_deg = float(self.get_parameter('offset_3_deg').value)
        self._off4_deg = float(self.get_parameter('offset_4_deg').value)
        self._L2_mm = float(self.get_parameter('link2_mm').value)
        self._L3_mm = float(self.get_parameter('link3_mm').value)
        self._elbow = self.get_parameter('elbow').get_parameter_value().string_value.strip().lower()

        self._ui_angle_space = self.get_parameter('ui_joint_angle_space').get_parameter_value().string_value.strip().lower()
        if self._ui_angle_space not in ('physical', 'motor', 'motor_simple'):
            self.get_logger().warn("ui_joint_angle_space must be 'physical', 'motor', or 'motor_simple'; defaulting to 'physical'")
            self._ui_angle_space = 'physical'

        self._pub = self.create_publisher(JointState, self._js_topic, 10)
        self._sub = self.create_subscription(TesterUICommand, self._cmd_topic, self._on_cmd, 10)

        # Published state is RAW/cumulative for joint2..4.
        # These are fed by multi_virtual_motor feedback.
        self._pos_raw = [0.0, 0.0, 0.0, 0.0]
        self._vel_raw = [0.0, 0.0, 0.0, 0.0]
        self._traj: _Trajectory | None = None  # legacy (kept for compatibility); no longer used for motion

        self._last_feedback_stamp_s: float | None = None

        self._motor_positions = [0.0, 0.0, 0.0, 0.0]
        self._motor_positions_valid = [False, False, False, False]

        self._motor_enabled = False
        self._operation_mode = 'pp'  # 'pp' | 'pvt'

        self._motor_prefixes = list(self.get_parameter('motor_prefixes').value)
        if len(self._motor_prefixes) != 4:
            self.get_logger().warn("motor_prefixes must be a list of length 4; defaulting to ['motor1','motor2','motor3','motor4']")
            self._motor_prefixes = ['motor1', 'motor2', 'motor3', 'motor4']

        # Publishers (to multi_virtual_motor)
        self._pub_mode: list = []
        self._pub_poscmd: list = []

        # Subscribers (from multi_virtual_motor)
        self._sub_gear: list = []

        # Service clients (to multi_virtual_motor)
        self._cli_enable: list = []
        self._cli_start: list = []
        self._cli_stop: list = []

        self._warned_service_missing: set[str] = set()

        # Keep references to one-shot timers so they don't get GC'ed before firing.
        self._one_shot_timers = []

        for i, prefix in enumerate(self._motor_prefixes):
            self._pub_mode.append(self.create_publisher(String, f'{prefix}/mode', 10))
            self._pub_poscmd.append(self.create_publisher(PositionCommand, f'{prefix}/position_cmd', 10))

            self._sub_gear.append(
                self.create_subscription(Float64, f'{prefix}/gear_position', lambda m, idx=i: self._on_motor_pos(idx, m), 10)
            )

            self._cli_enable.append(self.create_client(SetBool, f'{prefix}/set_enabled'))
            self._cli_start.append(self.create_client(Trigger, f'{prefix}/start_motion'))
            self._cli_stop.append(self.create_client(Trigger, f'{prefix}/stop_motion'))

        period = 1.0 / self._rate_hz if self._rate_hz > 0 else 0.02
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'robot_controller_virtual: cmd_topic={self._cmd_topic} -> publish {self._js_topic} @ {self._rate_hz:.1f} Hz; '
            f'joint1_input_unit={self._j1_unit}; gear_ratio={self._gear_ratio:g} '
            f'offsets=({self._off2_deg:g},{self._off3_deg:g},{self._off4_deg:g}) '
            f'links=({self._L2_mm:g}mm,{self._L3_mm:g}mm) elbow={self._elbow} '
            f'ui_joint_angle_space={self._ui_angle_space}'
        )

        self.get_logger().info(
            'multi_virtual_motor bridge enabled: publishing PositionCommand to '
            + ', '.join([f'{p}/position_cmd' for p in self._motor_prefixes])
        )

    def _on_motor_pos(self, idx: int, msg: Float64) -> None:
        # multi_virtual_motor publishes gear_position in degrees for revolute axes.
        self._motor_positions[idx] = float(msg.data)
        self._motor_positions_valid[idx] = True

    def _motor_deg_to_physical_deg(self, motor2_deg: float, motor3_deg: float, motor4_deg: float) -> tuple[float, float, float, float]:
        """Convert motor-side degrees into physical (gear/output) degrees.

        Matches the conventions in robot_controller/arm.py forward kinematics:
        theta2 = motor2/ratio + OFFSET_2
        theta3 = (motor3-motor2)/ratio + OFFSET_3
        yaw    = (-motor4)/ratio + OFFSET_4

        Returns: (theta2_deg, theta3_deg, yaw_deg, theta4_rel_deg)
        where theta4_rel is URDF joint_4 relative angle.
        """

        ratio = self._gear_ratio if abs(self._gear_ratio) > 1e-9 else 5.0
        theta2_deg = (float(motor2_deg) / ratio) + self._off2_deg
        theta3_deg = ((float(motor3_deg) - float(motor2_deg)) / ratio) + self._off3_deg
        yaw_deg = ((-float(motor4_deg)) / ratio) + self._off4_deg
        theta4_rel_deg = yaw_deg - theta2_deg - theta3_deg
        return theta2_deg, theta3_deg, yaw_deg, theta4_rel_deg

    def _physical_deg_to_raw(self, *, j1_m: float, theta2_deg: float, theta3_deg: float, theta4_rel_deg: float) -> list[float]:
        # Publish cumulative angles so joint_state_mapper can recover relative joint angles.
        j2_abs = _deg_to_rad(theta2_deg)
        j3_abs = _deg_to_rad(theta2_deg + theta3_deg)
        j4_abs = _deg_to_rad(theta2_deg + theta3_deg + theta4_rel_deg)
        return [float(j1_m), j2_abs, j3_abs, j4_abs]

    def _raw_to_rel(self, raw: list[float]) -> list[float]:
        # raw: [j1, j2_abs, j3_abs, j4_abs]
        j1 = float(raw[0])
        j2 = float(raw[1])
        j3_rel = float(raw[2]) - float(raw[1])
        j4_rel = float(raw[3]) - float(raw[2])
        return [j1, j2, j3_rel, j4_rel]

    def _rel_to_raw(self, rel: list[float]) -> list[float]:
        # rel: [j1, j2_abs, j3_rel, j4_rel]
        j1 = float(rel[0])
        j2 = float(rel[1])
        j3_abs = j2 + float(rel[2])
        j4_abs = j3_abs + float(rel[3])
        return [j1, j2, j3_abs, j4_abs]

    def _coor_to_raw(self, *, x_mm: float, y_mm: float, z_mm: float, yaw_deg: float) -> list[float] | None:
        # joint_1 (prismatic)
        if self._j1_unit == 'mm':
            j1_m = float(z_mm) / 1000.0
        elif self._j1_unit == 'm':
            j1_m = float(z_mm)
        else:
            j1_m = float(z_mm) / 1000.0

        L2 = float(self._L2_mm)
        L3 = float(self._L3_mm)
        if L2 <= 1e-9 or L3 <= 1e-9:
            self.get_logger().error('IK disabled: link2_mm and link3_mm must be > 0')
            return None

        x = float(x_mm)
        y = float(y_mm)

        # Clamp to reach.
        distance = math.hypot(x, y)
        max_reach = L2 + L3
        if distance > max_reach and distance > 1e-9:
            scale = max_reach / distance
            x *= scale
            y *= scale
            self.get_logger().warn('Target out of reach; clamped to max reach')

        cos_t3 = (x * x + y * y - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
        cos_t3 = _clamp(cos_t3, -1.0, 1.0)
        t3_rad = math.acos(cos_t3)
        if self._elbow == 'up':
            t3_rad = -t3_rad
        t3_deg = _rad_to_deg(t3_rad)

        k1 = L2 + L3 * cos_t3
        k2 = L3 * math.sin(t3_rad)
        t2_rad = math.atan2(y, x) - math.atan2(k2, k1)
        t2_deg = _rad_to_deg(t2_rad)

        theta4_rel_deg = float(yaw_deg) - t2_deg - t3_deg

        return self._physical_deg_to_raw(
            j1_m=j1_m,
            theta2_deg=t2_deg,
            theta3_deg=t3_deg,
            theta4_rel_deg=theta4_rel_deg,
        )

    def _convert_ui_joints_to_raw(self, joints: list[float]) -> list[float]:
        # UI: joints[0] is rail travel (mm by default)
        # UI: joints[1..3] are MOTOR angles (deg), matching robot_controller conventions.
        j = list(joints[:4])
        while len(j) < 4:
            j.append(0.0)

        j1_in = float(j[0])
        if self._j1_unit == 'mm':
            j1_m = j1_in / 1000.0
        elif self._j1_unit == 'm':
            j1_m = j1_in
        else:
            self.get_logger().warn(f"Unknown joint1_input_unit='{self._j1_unit}', defaulting to mm")
            j1_m = j1_in / 1000.0

        if self._ui_angle_space == 'motor':
            # Legacy / real-robot convention: UI joints[1..3] are motor-side degrees.
            theta2_deg, theta3_deg, _yaw_deg, theta4_rel_deg = self._motor_deg_to_physical_deg(
                float(j[1]), float(j[2]), float(j[3])
            )
        elif self._ui_angle_space == 'motor_simple':
            # Simple virtual convention: UI joints[1..3] are motor-side degrees.
            # Physical gear/output degrees are motor/ratio, without offsets/coupling.
            ratio = self._gear_ratio if abs(self._gear_ratio) > 1e-9 else 5.0
            theta2_deg = float(j[1]) / ratio
            theta3_deg = float(j[2]) / ratio
            theta4_rel_deg = float(j[3]) / ratio
        else:
            # Recommended for virtual pipeline: UI joints[1..3] are physical (gear/output) angles in degrees.
            # joints[1]=theta2_deg, joints[2]=theta3_deg, joints[3]=theta4_rel_deg
            theta2_deg = float(j[1])
            theta3_deg = float(j[2])
            theta4_rel_deg = float(j[3])

        return self._physical_deg_to_raw(
            j1_m=j1_m,
            theta2_deg=theta2_deg,
            theta3_deg=theta3_deg,
            theta4_rel_deg=theta4_rel_deg,
        )

    def _start_traj(self, goal_raw: list[float], duration_s: float, mode: str) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        duration = max(0.0, float(duration_s))

        if duration <= 1e-6:
            self._pos_raw = list(goal_raw)
            self._traj = None
            return

        self._traj = _Trajectory(
            start=list(self._pos_raw),
            goal=list(goal_raw),
            t0=now,
            duration=duration,
            mode=mode,
        )

    def _simulate_if_no_feedback(self) -> None:
        """Advance internal joint state when no motor feedback is present.

        This keeps RViz moving even when multi_virtual_motor isn't running.
        If motor feedback starts arriving, _on_timer will prefer that and effectively
        override this simulation.
        """

        if self._traj is None:
            return
        if any(self._motor_positions_valid):
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        t = max(0.0, now_s - float(self._traj.t0))
        duration = max(1e-9, float(self._traj.duration))
        alpha = min(1.0, t / duration)

        prev = list(self._pos_raw)
        self._pos_raw = [
            float(self._traj.start[i]) + (float(self._traj.goal[i]) - float(self._traj.start[i])) * alpha
            for i in range(4)
        ]

        # Approximate velocity from last published state.
        # This is good enough for visualization.
        if self._last_feedback_stamp_s is None:
            dt = 0.0
        else:
            dt = max(0.0, now_s - self._last_feedback_stamp_s)

        if dt > 1e-6:
            self._vel_raw = [(self._pos_raw[i] - prev[i]) / dt for i in range(4)]
        else:
            self._vel_raw = [0.0, 0.0, 0.0, 0.0]

        self._last_feedback_stamp_s = now_s

        if alpha >= 1.0:
            self._traj = None

    def _require_enabled(self, cmd: str) -> bool:
        if self._motor_enabled:
            return True
        self.get_logger().warn(
            f"Ignoring {cmd}: motor disabled. Send 'wake_up' first (enable motor)."
        )
        return False

    def _selected_motor_indices(self, motor_selector: str) -> list[int]:
        m = (motor_selector or '').strip().lower()
        if m == 'stepper_only':
            return [0]
        if m == 'servo_only':
            return [1, 2, 3]
        return [0, 1, 2, 3]

    def _ensure_service_ready(self, name: str, client) -> bool:
        if client.service_is_ready():
            return True
        try:
            # Avoid a common race: GUI sends wake_up/start immediately after launch,
            # but the multi_virtual_motor services aren't ready yet.
            client.wait_for_service(timeout_sec=1.0)
        except Exception:
            pass
        if client.service_is_ready():
            return True
        if name not in self._warned_service_missing:
            self._warned_service_missing.add(name)
            self.get_logger().warn(f"Service not available yet: {name}")
        return False

    def _call_set_enabled(self, idx: int, enabled: bool) -> None:
        prefix = self._motor_prefixes[idx]
        cli = self._cli_enable[idx]
        if not self._ensure_service_ready(f'{prefix}/set_enabled', cli):
            return
        req = SetBool.Request()
        req.data = bool(enabled)
        fut = cli.call_async(req)

        def _done(f):
            try:
                res = f.result()
                self.get_logger().info(f"{prefix}/set_enabled -> success={getattr(res,'success',None)} msg={getattr(res,'message','')}")
            except Exception as e:
                self.get_logger().warn(f"{prefix}/set_enabled call failed: {e}")

        fut.add_done_callback(_done)

    def _call_start(self, idx: int) -> None:
        prefix = self._motor_prefixes[idx]
        cli = self._cli_start[idx]
        if not self._ensure_service_ready(f'{prefix}/start_motion', cli):
            return
        req = Trigger.Request()
        fut = cli.call_async(req)

        def _done(f):
            try:
                res = f.result()
                self.get_logger().info(f"{prefix}/start_motion -> success={getattr(res,'success',None)} msg={getattr(res,'message','')}")
            except Exception as e:
                self.get_logger().warn(f"{prefix}/start_motion call failed: {e}")

        fut.add_done_callback(_done)

    def _call_stop(self, idx: int) -> None:
        prefix = self._motor_prefixes[idx]
        cli = self._cli_stop[idx]
        if not self._ensure_service_ready(f'{prefix}/stop_motion', cli):
            return
        req = Trigger.Request()
        fut = cli.call_async(req)

        def _done(f):
            try:
                res = f.result()
                self.get_logger().info(f"{prefix}/stop_motion -> success={getattr(res,'success',None)} msg={getattr(res,'message','')}")
            except Exception as e:
                self.get_logger().warn(f"{prefix}/stop_motion call failed: {e}")

        fut.add_done_callback(_done)

    def _start_after_delay(self, idx: int, delay_s: float = 0.05) -> None:
        # One-shot timer to call start_motion slightly after position_cmd publish.
        # This avoids a common race where start_motion arrives before the PositionCommand is processed.
        delay = max(0.0, float(delay_s))

        timer_ref = {'t': None}

        def _cb():
            try:
                self._call_start(idx)
            finally:
                t = timer_ref.get('t')
                if t is not None:
                    try:
                        t.cancel()
                    except Exception:
                        pass

        t = self.create_timer(delay, _cb)
        timer_ref['t'] = t
        self._one_shot_timers.append(t)

    def _send_position_command(self, idx: int, target: float, duration_s: float) -> None:
        # Ensure position_mode (multi_virtual_motor accepts mode via topic)
        self._pub_mode[idx].publish(String(data='position_mode'))

        # Try to shape motion time roughly to duration_s (approximate triangular profile)
        t = max(0.0, float(duration_s))
        if t <= 1e-3:
            t = 1e-3

        # IMPORTANT: PositionCommand.target_position is interpreted by the motor node as *gear_position*.
        # We treat gear_position as degrees for rotational axes.
        target_gear_units = _axis_raw_to_gear_units(idx, float(target))
        cur_gear_units = (
            float(self._motor_positions[idx])
            if self._motor_positions_valid[idx]
            else _axis_raw_to_gear_units(idx, float(self._pos_raw[idx]))
        )
        dist = abs(float(target_gear_units) - float(cur_gear_units))

        # If already close, keep defaults (let motor settle)
        if dist <= 1e-9:
            acc = 0.0
            dec = 0.0
            vmax = 0.0
        else:
            # Triangular-profile approximation:
            # a = 4*dist/t^2, vmax = a*t/2 = 2*dist/t
            acc = 4.0 * dist / (t * t)
            dec = acc
            vmax = 2.0 * dist / t

        msg = PositionCommand()
        msg.target_position = float(target_gear_units)
        msg.acceleration = float(acc)
        msg.deceleration = float(dec)
        msg.max_speed = float(vmax)
        self._pub_poscmd[idx].publish(msg)

    def _on_cmd(self, msg: TesterUICommand) -> None:
        cmd = (msg.command or '').strip()
        t_s = max(0.0, float(msg.time_ms) / 1000.0)

        selected = self._selected_motor_indices(getattr(msg, 'motor', 'all'))

        if cmd == 'wake_up':
            self._motor_enabled = True
            for i in range(4):
                self._call_set_enabled(i, True)
            self.get_logger().info('wake_up: motors enabled (multi_virtual_motor)')
            return

        if cmd == 'shutdown':
            self._motor_enabled = False
            self._traj = None
            for i in range(4):
                self._call_set_enabled(i, False)
            self.get_logger().info('shutdown: motors disabled (multi_virtual_motor)')
            return

        if cmd == 'stop':
            self._traj = None
            for i in range(4):
                self._call_stop(i)
            self.get_logger().info('stop: stop_motion sent to all motors')
            return

        if cmd == 'request_mode':
            self.get_logger().info(
                f"mode: enabled={self._motor_enabled} operation_mode={self._operation_mode}"
            )
            return

        # Normalize aliases
        if cmd == 'pp_move':
            cmd = 'pp_joint'
        if cmd == 'pvt_move':
            cmd = 'pvt_joint'

        if cmd in ('pp_joint', 'homing'):
            if not self._require_enabled(cmd):
                return

            self._operation_mode = 'pp'
            goal_raw = [0.0, 0.0, 0.0, 0.0] if cmd == 'homing' else self._convert_ui_joints_to_raw(list(msg.joints))

            # If motors aren't providing feedback, simulate motion in RViz.
            cur_rel = self._raw_to_rel(list(self._pos_raw))
            goal_rel = self._raw_to_rel(list(goal_raw))

            sim_rel = list(cur_rel)
            # joint_1 is independent
            if 0 in selected:
                sim_rel[0] = goal_rel[0]
            # joints 2..4 are coupled via cumulative/raw representation
            if 1 in selected:
                sim_rel[1] = goal_rel[1]
            if 2 in selected:
                sim_rel[2] = goal_rel[2]
            if 3 in selected:
                sim_rel[3] = goal_rel[3]

            sim_goal = self._rel_to_raw(sim_rel)
            self._start_traj(sim_goal, t_s, mode='pp')

            # Only command the selected motors; hold others.
            targets = list(self._pos_raw)
            for i in range(4):
                if i in selected:
                    targets[i] = goal_raw[i]
                else:
                    if self._motor_positions_valid[i]:
                        targets[i] = _maybe_deg_to_rad_for_axis(i, float(self._motor_positions[i]))

            for i in selected:
                # Idempotent: ensure motor is enabled on the motor-side.
                self._call_set_enabled(i, True)
                self._send_position_command(i, targets[i], t_s)
            for i in selected:
                self._start_after_delay(i, delay_s=0.05)

            self.get_logger().info(f'{cmd}: sent PositionCommand to motors={selected} duration={t_s:.3f}s targets={targets!r}')
            return

        if cmd == 'pvt_joint':
            if not self._require_enabled(cmd):
                return

            self._operation_mode = 'pvt'
            goal_raw = self._convert_ui_joints_to_raw(list(msg.joints))

            # If motors aren't providing feedback, simulate motion in RViz.
            cur_rel = self._raw_to_rel(list(self._pos_raw))
            goal_rel = self._raw_to_rel(list(goal_raw))

            sim_rel = list(cur_rel)
            if 0 in selected:
                sim_rel[0] = goal_rel[0]
            if 1 in selected:
                sim_rel[1] = goal_rel[1]
            if 2 in selected:
                sim_rel[2] = goal_rel[2]
            if 3 in selected:
                sim_rel[3] = goal_rel[3]

            sim_goal = self._rel_to_raw(sim_rel)
            self._start_traj(sim_goal, t_s, mode='pvt')

            # For now, drive multi_virtual_motor using position_mode as well.
            targets = list(self._pos_raw)
            for i in range(4):
                if i in selected:
                    targets[i] = goal_raw[i]
                else:
                    if self._motor_positions_valid[i]:
                        targets[i] = _maybe_deg_to_rad_for_axis(i, float(self._motor_positions[i]))

            for i in selected:
                self._call_set_enabled(i, True)
                self._send_position_command(i, targets[i], t_s)
            for i in selected:
                self._start_after_delay(i, delay_s=0.05)

            self.get_logger().info(f'{cmd}: sent PositionCommand (pvt request) motors={selected} duration={t_s:.3f}s targets={targets!r}')
            return

        if cmd in ('pp_coor', 'pvt_coor'):
            if not self._require_enabled(cmd):
                return

            mode = 'pp' if cmd == 'pp_coor' else 'pvt'
            self._operation_mode = mode

            x_mm, y_mm, z_mm, yaw_deg = list(msg.coor)[:4]
            goal_raw = self._coor_to_raw(x_mm=x_mm, y_mm=y_mm, z_mm=z_mm, yaw_deg=yaw_deg)
            if goal_raw is None:
                self.get_logger().error(f'{cmd}: IK failed; not moving')
                return

            self._start_traj(goal_raw, t_s, mode=mode)

            targets = list(self._pos_raw)
            for i in range(4):
                if i in selected:
                    targets[i] = goal_raw[i]
                else:
                    if self._motor_positions_valid[i]:
                        targets[i] = _maybe_deg_to_rad_for_axis(i, float(self._motor_positions[i]))

            for i in selected:
                self._call_set_enabled(i, True)
                self._send_position_command(i, targets[i], t_s)
            for i in selected:
                self._start_after_delay(i, delay_s=0.05)

            self.get_logger().info(
                f'{cmd}: mode={mode} duration={t_s:.3f}s coor={list(msg.coor)!r} targets={targets!r} motors={selected}'
            )
            return

        if cmd == 'read_position':
            self.get_logger().info(
                f'raw_position={self._pos_raw!r} motor_feedback={self._motor_positions!r} valid={self._motor_positions_valid!r}'
            )
            return

        if cmd == 'dancing':
            self.get_logger().warn('dancing: not implemented in robot_controller_virtual')
            return

        self.get_logger().warn(f'Unknown/unsupported command: {cmd!r}')

    def _on_timer(self) -> None:
        now_stamp = self.get_clock().now().to_msg()

        # Prefer feedback from multi_virtual_motor; fallback to last published.
        # Update per-axis so RViz moves even if only some topics are connected.
        if any(self._motor_positions_valid):
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if self._last_feedback_stamp_s is None:
                dt = 0.0
            else:
                dt = max(0.0, now_s - self._last_feedback_stamp_s)

            prev = list(self._pos_raw)
            for i in range(4):
                if self._motor_positions_valid[i]:
                    # Convert motor topic units (deg for revolute axes) back to JointState units (rad).
                    self._pos_raw[i] = _maybe_deg_to_rad_for_axis(i, float(self._motor_positions[i]))

            if dt > 1e-6:
                self._vel_raw = [(self._pos_raw[i] - prev[i]) / dt for i in range(4)]
            else:
                self._vel_raw = [0.0, 0.0, 0.0, 0.0]

            self._last_feedback_stamp_s = now_s

        # If there is no motor feedback, advance our internal simulated state.
        self._simulate_if_no_feedback()

        js = JointState()
        js.header.stamp = now_stamp
        js.name = [self._j1, self._j2, self._j3, self._j4]
        js.position = list(self._pos_raw)
        js.velocity = list(self._vel_raw)
        self._pub.publish(js)


def main() -> None:
    rclpy.init()
    node = VirtualRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except AttributeError:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
