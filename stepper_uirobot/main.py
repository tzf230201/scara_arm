"""
main.py — One-way ZMQ subscriber with ROS-like rate + callbacks (print-only)
- SUB to commands (JSON) sent by gui.py
- 50 Hz control loop (anti-drift) — placeholder for future logic
- Callbacks just print the command and key parameters (including `motor`)

Run:
    pip install pyzmq
    # Make sure endpoint matches gui.py (default IPC)
    python main.py

Config env vars:
    CMD_ENDPOINT  (default: ipc:///tmp/motor_cmd)
    TARGET_HZ     (default: 50)
"""
import os
import time
import zmq

# ---- Config ----
CMD_ENDPOINT = os.getenv("CMD_ENDPOINT", "ipc:///tmp/motor_cmd")
TARGET_HZ = float(os.getenv("TARGET_HZ", 50))
Ts = 1.0 / TARGET_HZ

# ---- Helper: ROS-like Rate ----
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
            # overrun: resync so we don't drift forever
            self.next_t = time.perf_counter()

# ---- ZMQ SUB setup ----
ctx = zmq.Context.instance()
sub = ctx.socket(zmq.SUB)
sub.setsockopt_string(zmq.SUBSCRIBE, "")
sub.setsockopt(zmq.RCVHWM, 1000)
sub.connect(CMD_ENDPOINT)
# slow-joiner: give time to connect to PUB
time.sleep(0.2)

poller = zmq.Poller()
poller.register(sub, zmq.POLLIN)

# ---- Optional state (for future) ----
state = {"running": True}

# ---- Callbacks (print-only) ----

def cb(tag: str, msg: dict):
    motor = msg.get("motor", "-")
    print(f"[cb] {tag}  motor={motor}  payload={msg}")

# high-level actions
HANDLERS = {
    "wake_up":        lambda m: cb("wake_up", m),
    "shutdown":       lambda m: cb("shutdown", m),
    "stop":           lambda m: cb("stop", m),
    "homing":         lambda m: cb("homing", m),
    "dancing":        lambda m: cb("dancing", m),
    "read_position":  lambda m: cb("read_position", m),
    "read_encoder":   lambda m: cb("read_encoder", m),
    "set_origin":     lambda m: cb("set_origin", m),
    # motion profiles
    "pp_joint":       lambda m: cb("pp_joint", m),
    "pp_coor":        lambda m: cb("pp_coor", m),
    "pvt_joint":      lambda m: cb("pvt_joint", m),
    "pvt_coor":       lambda m: cb("pvt_coor", m),
    # lifecycle
    "quit":           lambda m: (print("[cb] quit"), state.update(running=False)),
}

# ---- Periodic control step (placeholder) ----

def control_step():
    # Put your 50 Hz logic here later (e.g., watchdogs, soft realtime checks)
    pass

# ---- Main loop ----
if __name__ == "__main__":
    print(f"[main] SUB on {CMD_ENDPOINT}  rate={TARGET_HZ} Hz")
    rate = Rate(TARGET_HZ)
    try:
        while state["running"]:
            # 1) pump incoming commands (non-blocking)
            for _ in range(6):  # small batch to keep latency low
                if not poller.poll(0):
                    break
                try:
                    msg = sub.recv_json(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                cmd = msg.get("command")
                fn = HANDLERS.get(cmd)
                if fn:
                    fn(msg)
                else:
                    print("[warn] unknown command:", msg)

            # 2) periodic step
            control_step()

            # 3) maintain period
            rate.sleep()
    except KeyboardInterrupt:
        print("[main] KeyboardInterrupt")
    finally:
        sub.close(0)
        ctx.term()
        print("[main] exit")
