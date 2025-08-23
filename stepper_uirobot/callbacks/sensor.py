# callbacks/sensors.py — baca/kalibrasi (print-only)
def _motor_of(msg): return msg.get("motor", "-")

def read_position(msg, state):
    print(f"[cb] read_position motor={_motor_of(msg)} -> pos_abs={state.get('pos_abs')}")

def read_encoder(msg, state):
    print(f"[cb] read_encoder motor={_motor_of(msg)}")

def set_origin(msg, state):
    print(f"[cb] set_origin  motor={_motor_of(msg)}")
