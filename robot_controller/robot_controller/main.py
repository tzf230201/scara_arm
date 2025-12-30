"""main.py — ROS 2 subscriber for Tester UI commands.

This replaces the old ZMQ SUB loop.

Listens on /scara_arm/tester_ui (scara_arm_msgs/TesterUICommand) and routes
commands to callbacks.HANDLERS, while running callbacks.routine at TARGET_HZ.
"""

import os
import time

import rclpy
from rclpy.node import Node

from scara_arm_msgs.msg import TesterUICommand

from .callbacks import routine, HANDLERS


DEFAULT_TOPIC = '/scara_arm/tester_ui'
TARGET_HZ = float(os.getenv('TARGET_HZ', 40))


def _msg_to_dict(msg: TesterUICommand) -> dict:
    return {
        'command': msg.command,
        'motor': msg.motor,
        'time': int(msg.time_ms),
        'joints': list(msg.joints),
        'coor': list(msg.coor),
        'pvt_path': msg.pvt_path,
        # keep stamp if needed later
        'stamp': msg.stamp,
    }


class ScaraArmController(Node):
    def __init__(self) -> None:
        super().__init__('scara_arm_controller')

        topic = os.getenv('TESTER_UI_TOPIC', DEFAULT_TOPIC)
        self._topic = topic

        # Shared state for callbacks
        self.state = {
            'running': True,
            'motor_on': False,
            'routine': False,
            'last_cmd_ts': time.time(),
        }

        self._sub = self.create_subscription(
            TesterUICommand,
            self._topic,
            self._on_cmd,
            10,
        )

        period = 1.0 / TARGET_HZ if TARGET_HZ > 0 else 0.025
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f'Listening: {self._topic} @ {TARGET_HZ:.1f} Hz')

    def _on_cmd(self, msg: TesterUICommand) -> None:
        cmd_dict = _msg_to_dict(msg)
        cmd = cmd_dict.get('command')
        fn = HANDLERS.get(cmd)
        if fn is None:
            self.get_logger().warn(f'Unknown command: {cmd_dict!r}')
            return

        try:
            fn(cmd_dict, self.state)
            self.state['last_cmd_ts'] = time.time()
        except Exception as e:
            self.get_logger().error(f'Handler failed for command={cmd}: {e}')

    def _on_timer(self) -> None:
        try:
            routine(self.state)
        except Exception as e:
            self.get_logger().error(f'Routine error: {e}')

        if not self.state.get('running', True):
            self.get_logger().info('Stopping (state.running=false)')
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = ScaraArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
