# callbacks/motion.py — perintah gerak (print-only)
def _motor_of(msg): return msg.get("motor", "-")

def pp_joint(msg, state):
    print(f"[cb] pp_joint  motor={_motor_of(msg)} joints={msg.get('joints')} time={msg.get('time')}")

def pp_coor(msg, state):
    print(f"[cb] pp_coor   motor={_motor_of(msg)} coor={msg.get('coor')} time={msg.get('time')}")

def pvt_joint(msg, state):
    print(f"[cb] pvt_joint motor={_motor_of(msg)} joints={msg.get('joints')} time={msg.get('time')}")

def pvt_coor(msg, state):
    print(f"[cb] pvt_coor  motor={_motor_of(msg)} coor={msg.get('coor')} time={msg.get('time')}")
