# callbacks/__init__.py — satukan handler jadi satu dict
from .system  import wake_up, shutdown, stop, homing, dancing, quit
from .motion  import pp_joint, pp_coor, pvt_joint, pvt_coor
from .sensors import read_position, read_encoder, set_origin

HANDLERS = {
    # system
    "wake_up":       wake_up,
    "shutdown":      shutdown,
    "stop":          stop,
    "homing":        homing,
    "dancing":       dancing,
    "quit":          quit,
    # motion
    "pp_joint":      pp_joint,
    "pp_coor":       pp_coor,
    "pvt_joint":     pvt_joint,
    "pvt_coor":      pvt_coor,
    # sensors
    "read_position": read_position,
    "read_encoder":  read_encoder,
    "set_origin":    set_origin,
}
