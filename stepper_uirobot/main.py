import os, time, zmq

CMD_ENDPOINT = os.getenv("CMD_ENDPOINT", "ipc:///tmp/motor_cmd")
TARGET_HZ = float(os.getenv("TARGET_HZ", 50))
Ts = 1.0 / TARGET_HZ

class Rate:
    def __init__(self, hz): self.T, self.next_t = 1.0/hz, time.perf_counter()
    def sleep(self):
        self.next_t += self.T
        d = self.next_t - time.perf_counter()
        time.sleep(d) if d > 0 else setattr(self, "next_t", time.perf_counter())

ctx = zmq.Context.instance()
sub = ctx.socket(zmq.SUB)
sub.setsockopt_string(zmq.SUBSCRIBE, "")
sub.setsockopt(zmq.RCVHWM, 1000)
sub.connect(CMD_ENDPOINT)
time.sleep(0.2)  # slow-joiner

poller = zmq.Poller()
poller.register(sub, zmq.POLLIN)

state = {"pos_abs": 0.0, "speed": 0.0, "running": True}

# callbacks — semua menampilkan 'motor' jika ada
def cb_print(tag, msg): print(f"[cb] {tag} motor={msg.get('motor','-')} payload={msg}")

def cb_set_speed(msg):
    try: state["speed"] = float(msg.get("pps", 0))
    except: state["speed"] = 0.0
    print(f"[cb] set_speed: {state['speed']}  motor={msg.get('motor','-')}")

HANDLERS = {
    "wake_up":        lambda m: cb_print("wake_up", m),
    "shutdown":       lambda m: cb_print("shutdown", m),
    "stop":           lambda m: cb_print("stop", m),
    "homing":         lambda m: cb_print("homing", m),
    "read_position":  lambda m: cb_print("read_position", m),
    "pp_joint":       lambda m: cb_print("pp_joint", m),
    "pp_coor":        lambda m: cb_print("pp_coor", m),
    "pvt_joint":      lambda m: cb_print("pvt_joint", m),
    "pvt_coor":       lambda m: cb_print("pvt_coor", m),
    "set_speed":      cb_set_speed,
    "quit":           lambda m: (print("[cb] quit"), state.update(running=False)),
}

def control_step(): state["pos_abs"] += state["speed"] * Ts

if __name__ == "__main__":
    print(f"[main] SUB on {CMD_ENDPOINT}, rate={TARGET_HZ} Hz")
    rate = Rate(TARGET_HZ)
    try:
        while state["running"]:
            for _ in range(6):
                if not poller.poll(0): break
                try:
                    msg = sub.recv_json(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                fn = HANDLERS.get(msg.get("command"))
                fn(msg) if fn else print("[warn] unknown:", msg)
            control_step()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        sub.close(0); ctx.term()
        print("[main] exit")
