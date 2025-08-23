# main.py — ZMQ SUB (one-way), ROS-like rate + router ke callbacks
import os, time, zmq
from callbacks import HANDLERS  # mapping: "command" -> fungsi callback

CMD_ENDPOINT = os.getenv("CMD_ENDPOINT", "ipc:///tmp/motor_cmd")
TARGET_HZ = float(os.getenv("TARGET_HZ", 50))
Ts = 1.0 / TARGET_HZ

class Rate:
    def __init__(self, hz): self.T, self.next_t = 1.0/hz, time.perf_counter()
    def sleep(self):
        self.next_t += self.T
        dt = self.next_t - time.perf_counter()
        time.sleep(dt) if dt > 0 else setattr(self, "next_t", time.perf_counter())

ctx = zmq.Context.instance()
sub = ctx.socket(zmq.SUB)
sub.setsockopt_string(zmq.SUBSCRIBE, "")
sub.setsockopt(zmq.RCVHWM, 1000)
sub.connect(CMD_ENDPOINT)
time.sleep(0.2)  # slow-joiner

poller = zmq.Poller(); poller.register(sub, zmq.POLLIN)

# State bersama buat semua callback (boleh kamu tambah field lain)
state = {
    "running": True,
    "motor_on": False,
    "pos_abs": 0.0,
    "speed": 0.0,
    "last_cmd_ts": time.time(),
}

def control_step():
    # contoh ringan: integrasi posisi + heartbeat 1 Hz
    state["pos_abs"] += state["speed"] * Ts
    if not hasattr(control_step, "_acc"): control_step._acc = 0.0
    control_step._acc += Ts
    if control_step._acc >= 1.0:
        print(f"[hb] pos={state['pos_abs']:.1f} speed={state['speed']:.1f} motor_on={state['motor_on']}")
        control_step._acc = 0.0

if __name__ == "__main__":
    print(f"[main] SUB on {CMD_ENDPOINT}  rate={TARGET_HZ} Hz")
    rate = Rate(TARGET_HZ)
    try:
        while state["running"]:
            # 1) proses pesan (non-blocking batch kecil)
            for _ in range(6):
                if not poller.poll(0): break
                try:
                    msg = sub.recv_json(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                cmd = msg.get("command")
                fn = HANDLERS.get(cmd)
                if fn:
                    fn(msg, state)   # <— panggil callback di file terpisah
                    state["last_cmd_ts"] = time.time()
                else:
                    print("[warn] unknown command:", msg)

            # 2) tugas periodik
            control_step()

            # 3) pertahankan periode
            rate.sleep()
    except KeyboardInterrupt:
        print("[main] KeyboardInterrupt")
    finally:
        sub.close(0); ctx.term(); print("[main] exit")
