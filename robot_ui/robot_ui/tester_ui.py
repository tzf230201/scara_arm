import os
import tkinter as tk
from tkinter import filedialog

import rclpy
from rclpy.node import Node

from scara_arm_msgs.msg import TesterUICommand


DEFAULT_TOPIC = '/scara_arm/tester_ui'


def _f2(x: str, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return float(default)


def _i2(x: str, default: int = 0) -> int:
    try:
        return int(float(x))
    except Exception:
        return int(default)


def _pad4(values: list[float]) -> list[float]:
    out = list(values[:4])
    while len(out) < 4:
        out.append(0.0)
    return out


class TesterUiRosPublisher(Node):
    def __init__(self, topic_name: str) -> None:
        super().__init__('tester_ui')
        self._topic_name = topic_name
        self._pub = self.create_publisher(TesterUICommand, self._topic_name, 10)

    def publish_command(
        self,
        command: str,
        motor: str,
        time_ms: int,
        joints: list[float] | None = None,
        coor: list[float] | None = None,
        csv_path: str = '',
    ) -> None:
        msg = TesterUICommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = command
        msg.motor = motor
        msg.time_ms = int(time_ms)
        msg.joints = _pad4(joints or [0.0, 0.0, 0.0, 0.0])
        msg.coor = _pad4(coor or [0.0, 0.0, 0.0, 0.0])
        msg.csv_path = csv_path or ''

        self._pub.publish(msg)
        self.get_logger().info(f'Published {command} to {self._topic_name}')


def main() -> None:
    topic_name = os.getenv('TESTER_UI_TOPIC', DEFAULT_TOPIC)

    rclpy.init()
    node = TesterUiRosPublisher(topic_name)

    root = tk.Tk()
    root.title('Motor Control Panel (ROS 2)')

    motor_type = tk.StringVar(value='all')
    selected_csv_path = tk.StringVar(value='')

    def current_motor() -> str:
        return motor_type.get()

    # --- Motor selection ---
    radio = tk.LabelFrame(root, text='Motor Selection', padx=10, pady=5)
    radio.grid(row=0, column=0, columnspan=2, pady=5, sticky='ew')
    tk.Radiobutton(radio, text='All motors', variable=motor_type, value='all').grid(row=0, column=0)
    tk.Radiobutton(radio, text='Stepper only', variable=motor_type, value='stepper_only').grid(row=0, column=1)
    tk.Radiobutton(radio, text='Servo only', variable=motor_type, value='servo_only').grid(row=0, column=2)

    # --- Travel time ---
    tk.Label(root, text='Travel time (ms):').grid(row=1, column=0)
    entry_time = tk.Entry(root)
    entry_time.insert(0, '4000')
    entry_time.grid(row=1, column=1)

    # --- Joint inputs ---
    entries_joint: list[tk.Entry] = []
    for i in range(4):
        tk.Label(root, text=f'Joint {i + 1} (deg):').grid(row=2 + i, column=0)
        e = tk.Entry(root)
        e.insert(0, '0')
        e.grid(row=2 + i, column=1)
        entries_joint.append(e)

    # --- Coordinate inputs ---
    labels = ['X (mm):', 'Y (mm):', 'Z (mm):', 'Yaw (deg):']
    entries_coor: list[tk.Entry] = []
    for i, label in enumerate(labels):
        tk.Label(root, text=label).grid(row=6 + i, column=0)
        e = tk.Entry(root)
        e.insert(0, '0')
        e.grid(row=6 + i, column=1)
        entries_coor.append(e)

    # Status label
    lbl_last = tk.Label(root, text=f'topic: {topic_name}')
    lbl_last.grid(row=18, column=0, columnspan=2, sticky='w')

    # ---- helpers ----
    def send(cmd: str, *, joints: list[float] | None = None, coor: list[float] | None = None, csv_path: str = '') -> None:
        node.publish_command(
            command=cmd,
            motor=current_motor(),
            time_ms=_i2(entry_time.get(), 0),
            joints=joints,
            coor=coor,
            csv_path=csv_path,
        )
        lbl_last.config(text=f'last cmd: {cmd} -> {topic_name}')

    # ---- Button handlers ----
    def send_pp_joint() -> None:
        joints = [_f2(e.get(), 0) for e in entries_joint]
        send('pp_joint', joints=joints)

    def send_pp_coor() -> None:
        coor = [_f2(e.get(), 0) for e in entries_coor]
        send('pp_coor', coor=coor)

    def send_pvt_joint() -> None:
        joints = [_f2(e.get(), 0) for e in entries_joint]
        send('pvt_joint', joints=joints)

    def send_pvt_coor() -> None:
        coor = [_f2(e.get(), 0) for e in entries_coor]
        send('pvt_coor', coor=coor)

    def choose_csv() -> None:
        file_path = filedialog.askopenfilename(
            title='Choose CSV file',
            filetypes=[('CSV files', '*.csv'), ('All files', '*.*')],
        )
        if file_path:
            selected_csv_path.set(file_path)
            lbl_last.config(text=f'CSV chosen: {os.path.basename(file_path)}')

    def request_mode() -> None:
        send('request_mode')

    def send_dancing() -> None:
        send('dancing', csv_path=selected_csv_path.get())

    # ---- Buttons ----
    tk.Button(root, text='Wake Up', bg='purple', fg='white', command=lambda: send('wake_up')).grid(
        row=10, column=0, sticky='ew'
    )
    tk.Button(root, text='Shutdown', bg='maroon', fg='white', command=lambda: send('shutdown')).grid(
        row=10, column=1, sticky='ew'
    )

    tk.Button(root, text='PP Joint', command=send_pp_joint).grid(row=11, column=0, sticky='ew')
    tk.Button(root, text='PP Coor', command=send_pp_coor).grid(row=11, column=1, sticky='ew')

    tk.Button(root, text='PVT Joint', command=send_pvt_joint).grid(row=12, column=0, sticky='ew')
    tk.Button(root, text='PVT Coor', command=send_pvt_coor).grid(row=12, column=1, sticky='ew')

    tk.Button(root, text='Read Position', bg='orange', command=lambda: send('read_position')).grid(
        row=13, column=0, sticky='ew'
    )
    tk.Button(root, text='Dancing', bg='green', fg='white', command=send_dancing).grid(row=13, column=1, sticky='ew')

    tk.Button(root, text='Homing', bg='cyan', command=lambda: send('homing')).grid(row=14, column=0, sticky='ew')
    tk.Button(root, text='Stop', bg='red', fg='white', command=lambda: send('stop')).grid(row=14, column=1, sticky='ew')

    tk.Button(root, text='Read Encoder', command=lambda: send('read_encoder')).grid(row=15, column=0, sticky='ew')
    tk.Button(root, text='Set Origin', command=lambda: send('set_origin')).grid(row=15, column=1, sticky='ew')

    tk.Button(root, text='OUT2 On', command=lambda: send('out2_active')).grid(row=16, column=0, sticky='ew')
    tk.Button(root, text='OUT2 Off', command=lambda: send('out2_nonactive')).grid(row=16, column=1, sticky='ew')

    tk.Button(root, text='Choose CSV', bg='lightblue', command=choose_csv).grid(row=17, column=0, sticky='ew')
    tk.Button(root, text='Request Mode', bg='lightgreen', command=request_mode).grid(row=17, column=1, sticky='ew')

    def _spin_ros_once() -> None:
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            # Keep UI alive even if DDS hiccups.
            pass
        root.after(20, _spin_ros_once)

    def _on_close() -> None:
        try:
            node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        root.destroy()

    root.after(20, _spin_ros_once)
    root.protocol('WM_DELETE_WINDOW', _on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
