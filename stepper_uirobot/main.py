"""
main.py — One-way ZMQ subscriber with ROS-like rate + callbacks
- SUBscribes to commands (JSON) and routes to print-only callbacks
- 50 Hz control loop (anti-drift scheduler)
- No telemetry publishing (one-way only)

Run:
    pip install pyzmq
    # ensure your GUI is running and PUB-bound to the same endpoint
    python main.py

Config:
    CMD_ENDPOINT env var (default: ipc:///tmp/motor_cmd)
    Example TCP:
        export CMD_ENDPOINT=tcp://127.0.0.1:5555
        python main.py
"""
import os
import time
import zmq

CMD_ENDPOINT = os.getenv("CMD_ENDPOINT", "ipc:///tmp/motor_cmd")
TARGET_HZ = float(os.getenv("TARGET_HZ", 50))
Ts = 1.0 / TARGET_HZ

class Rate:
    def __init__(self, hz: float):
        self.T = 1.0 / hz
        self.next_t = time.perf_counter()
    def sleep(self):
        self.next_t += self.T
        dt = self.next_t - time.perf_counter()
        if dt > 0:
            time.sleep(dt)
        else:
            # overrun; resync so we don't drift forever
            self.next_t = time.perf_counter()

# ---- ZMQ setup ----
ctx = zmq.Context.instance()
sub = ctx.socket(zmq.SUB)
sub.setsockopt_string(zmq.SUBSCRIBE, "")
sub.setsockopt(zmq.RCVHWM, 1000)
sub.connect(CMD_ENDPOINT)

# slow-joiner guard: give time to connect before first send from GUI
time.sleep(0.2)

poller = zmq.Poller()
poller.register(sub, zmq.POLLIN)

# ---- State (optional, for demo) ----
state = {
    "motor_on": False,
    "pos_abs": 0.0,
    "speed": 0.0,
    "running": True,
}

# ---- Callbacks (print-only) ----
def cb_motor_selection(msg):
    print("[cb] motor_selection:", msg.get("motor"))

def cb_wake_up(msg):
    state["motor_on"] = True
    print("[cb] wake_up")

def cb_shutdown(msg):
    state["motor_on"] = False
    state["speed"] = 0.0
    print("[cb] shutdown")

def cb_stop(msg):
    state["speed"] = 0.0
    print("[cb] stop")

def cb_homing(msg):
    state["pos_abs"] = 0.0
    state["speed"] = 0.0
    print("[cb] homing")

def cb_read_position(msg):
    print("[cb] read_position (demo) -> pos_abs:", state["pos_abs"])

def cb_set_speed(msg):
    try:
        pps = float(msg.get("pps", 0))
    except Exception:
        pps = 0.0
    state["speed"] = pps
    print(f"[cb] set_speed: {pps} pps")

def cb_pp_joint(msg):
    print("[cb] pp_joint:", msg.get("joints"), "time:", msg.get("time"))

def cb_pp_coor(msg):
    print("[cb] pp_coor:", msg.get("coor"), "time:", msg.get("time"))

def cb_pvt_joint(msg):
    print("[cb] pvt_joint:", msg.get("joints"), "time:", msg.get("time"))

def cb_pvt_coor(msg):
    print("[cb] pvt_coor:", msg.get("coor"), "time:", msg.get("time"))

def cb_quit(msg):
    state["running"] = False
    print("[cb] quit")

HANDLERS = {
    "motor_selection": cb_motor_selection,
    "wake_up": cb_wake_up,
    "shutdown": cb_shutdown,
    "stop": cb_stop,
    "homing": cb_homing,
    "read_position": cb_read_position,
    "set_speed": cb_set_speed,
    "pp_joint": cb_pp_joint,
    "pp_coor": cb_pp_coor,
    "pvt_joint": cb_pvt_joint,
    "pvt_coor": cb_pvt_coor,
    "quit": cb_quit,
}

# ---- Periodic control loop (ROS-like) ----
def control_step():
    # Quiet control step; simulate position integration
    state["pos_abs"] += state["speed"] * Ts

if __name__ == "__main__":
    print(f"[main] started: CMD={CMD_ENDPOINT}, rate={TARGET_HZ} Hz")
    rate = Rate(TARGET_HZ)
    try:
        while state["running"]:
            # 1) Process pending commands (non-blocking)
            for _ in range(6):
                if poller.poll(0):
                    try:
                        msg = sub.recv_json(flags=zmq.NOBLOCK)
                        cmd = msg.get("command")
                        fn = HANDLERS.get(cmd)
                        if fn:
                            fn(msg)
                        else:
                            print("[warn] unknown command:", msg)
                    except zmq.Again:
                        break
                else:
                    break

            # 2) Run periodic control step (50 Hz default)
            control_step()

            # 3) Sleep to maintain target period
            rate.sleep()
    except KeyboardInterrupt:
        print("[main] KeyboardInterrupt")
    finally:
        sub.close(0)
        ctx.term()
        print("[main] exit")
