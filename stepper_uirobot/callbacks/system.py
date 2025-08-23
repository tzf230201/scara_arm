# callbacks/system.py — aksi sistem (print-only prototyping)
def _motor_of(msg): return msg.get("motor", "-")

def wake_up(msg, state):
    state["motor_on"] = True
    print(f"[cb] wake_up   motor={_motor_of(msg)}")

def shutdown(msg, state):
    state["motor_on"] = False
    state["speed"] = 0.0
    print(f"[cb] shutdown  motor={_motor_of(msg)}")

def stop(msg, state):
    state["speed"] = 0.0
    print(f"[cb] stop      motor={_motor_of(msg)}")

def homing(msg, state):
    state["pos_abs"] = 0.0
    state["speed"] = 0.0
    print(f"[cb] homing    motor={_motor_of(msg)}")

def dancing(msg, state):
    # nanti bisa diisi sekuens gerak; sekarang print
    print(f"[cb] dancing   motor={_motor_of(msg)}")

def quit(msg, state):
    state["running"] = False
    print(f"[cb] quit      motor={_motor_of(msg)}")
