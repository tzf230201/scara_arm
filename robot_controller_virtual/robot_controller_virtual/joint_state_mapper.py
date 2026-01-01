import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMapper(Node):
    """Map cumulative/raw JointState into relative joints for robot_state_publisher.

    Expected semantics (names configurable):
    - joint_1: pass-through
    - joint_2: pass-through
    - joint_3_out = joint_3_raw - joint_2_raw
    - joint_4_out = joint_4_raw - joint_3_raw

    This matches the convention used by the virtual controller, where joint2..4
    are published as cumulative motor feedback angles.
    """

    def __init__(self) -> None:
        super().__init__('joint_state_mapper')

        self.declare_parameter('input_topic', '/joint_states_raw')
        self.declare_parameter('output_topic', '/joint_states')

        self.declare_parameter('joint1_name', 'joint_1')
        self.declare_parameter('joint2_name', 'joint_2')
        self.declare_parameter('joint3_name', 'joint_3')
        self.declare_parameter('joint4_name', 'joint_4')

        self.declare_parameter('epsilon', 1e-9)
        self.declare_parameter('publish_sparse', False)
        self.declare_parameter('publish_full_on_first_msg', True)
        self.declare_parameter('force_stamp_now_if_zero', True)

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self._j1 = self.get_parameter('joint1_name').get_parameter_value().string_value
        self._j2 = self.get_parameter('joint2_name').get_parameter_value().string_value
        self._j3 = self.get_parameter('joint3_name').get_parameter_value().string_value
        self._j4 = self.get_parameter('joint4_name').get_parameter_value().string_value

        self._epsilon = float(self.get_parameter('epsilon').value)
        self._publish_sparse = bool(self.get_parameter('publish_sparse').value)
        self._publish_full_on_first_msg = bool(self.get_parameter('publish_full_on_first_msg').value)
        self._force_stamp_now_if_zero = bool(self.get_parameter('force_stamp_now_if_zero').value)

        self._pub = self.create_publisher(JointState, self._output_topic, 10)
        self._sub = self.create_subscription(JointState, self._input_topic, self._on_joint_state, 10)

        self.get_logger().info(
            f"Mapping JointState: {self._j1}={self._j1}, {self._j2}={self._j2}, "
            f"{self._j3}=-{self._j2}+{self._j3}, {self._j4}=-{self._j3}+{self._j4} | "
            f"{self._input_topic} -> {self._output_topic}"
        )

        self._warned_missing = False
        self._warned_incomplete_position = False
        self._has_published_once = False
        self._last_out_position_by_name: dict[str, float] = {}
        self._last_out_velocity_by_name: dict[str, float] = {}

    def _on_joint_state(self, msg: JointState) -> None:
        name_to_index = {name: i for i, name in enumerate(msg.name)}

        if len(msg.name) > 0 and len(msg.position) != len(msg.name):
            if not self._warned_incomplete_position:
                self.get_logger().warn(
                    'JointStateMapper: received JointState with msg.name but msg.position length != len(msg.name). '
                    'Dropping message to avoid zero-flicker. Ensure only one JointState publisher is active and that it publishes full arrays.'
                )
                self._warned_incomplete_position = True
            return

        required = [self._j1, self._j2, self._j3, self._j4]
        missing = [j for j in required if j not in name_to_index]
        if missing:
            if not self._warned_missing:
                self.get_logger().warn(
                    'JointStateMapper: missing joints in msg.name: ' + ', '.join(missing) + '. Passing message through unchanged.'
                )
                self._warned_missing = True
            self._pub.publish(msg)
            return

        out = JointState()
        out.header = msg.header

        if self._force_stamp_now_if_zero:
            if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
                out.header.stamp = self.get_clock().now().to_msg()

        out.name = list(msg.name)

        pos_raw = list(msg.position) if msg.position else []
        vel_raw = list(msg.velocity) if msg.velocity else []
        eff = list(msg.effort) if msg.effort else []

        # Positions
        if pos_raw and len(pos_raw) == len(out.name):
            i1 = name_to_index[self._j1]
            i2 = name_to_index[self._j2]
            i3 = name_to_index[self._j3]
            i4 = name_to_index[self._j4]

            pos = list(pos_raw)
            pos[i3] = pos_raw[i3] - pos_raw[i2]
            pos[i4] = pos_raw[i4] - pos_raw[i3]
            out.position = pos

        # Velocities (if present and aligned)
        if vel_raw and len(vel_raw) == len(out.name):
            i2 = name_to_index[self._j2]
            i3 = name_to_index[self._j3]
            i4 = name_to_index[self._j4]

            vel = list(vel_raw)
            vel[i3] = vel_raw[i3] - vel_raw[i2]
            vel[i4] = vel_raw[i4] - vel_raw[i3]
            out.velocity = vel

        if eff and len(eff) == len(out.name):
            out.effort = eff

        # Optionally publish sparse updates
        if self._publish_sparse and self._has_published_once:
            sparse = JointState()
            sparse.header = out.header

            changed_names: list[str] = []
            changed_positions: list[float] = []
            changed_velocities: list[float] = []

            for i, name in enumerate(out.name):
                p = out.position[i] if len(out.position) == len(out.name) else None
                v = out.velocity[i] if len(out.velocity) == len(out.name) else None

                prev_p = self._last_out_position_by_name.get(name)
                prev_v = self._last_out_velocity_by_name.get(name)

                pos_changed = p is not None and (prev_p is None or abs(p - prev_p) > self._epsilon)
                vel_changed = v is not None and (prev_v is None or abs(v - prev_v) > self._epsilon)

                if pos_changed or vel_changed:
                    changed_names.append(name)
                    if p is not None:
                        changed_positions.append(p)
                    if v is not None:
                        changed_velocities.append(v)

            if changed_names:
                sparse.name = changed_names
                if changed_positions and len(changed_positions) == len(changed_names):
                    sparse.position = changed_positions
                if changed_velocities and len(changed_velocities) == len(changed_names):
                    sparse.velocity = changed_velocities
                self._pub.publish(sparse)
        else:
            self._pub.publish(out)

        if len(out.position) == len(out.name):
            self._last_out_position_by_name = {n: out.position[i] for i, n in enumerate(out.name)}
        if len(out.velocity) == len(out.name):
            self._last_out_velocity_by_name = {n: out.velocity[i] for i, n in enumerate(out.name)}

        if self._publish_full_on_first_msg and not self._has_published_once:
            self._has_published_once = True
        elif not self._publish_sparse:
            self._has_published_once = True


def main() -> None:
    rclpy.init()
    node = JointStateMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
