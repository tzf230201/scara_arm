# callbacks.py — ZMQ worker callbacks (single file)
# - Fokus: wake_up, shutdown, stop, read_position, read_encoder untuk IDs 6/7/8 (via PA)
# - Perintah lain stub (print) agar main.py tetap bersih

from __future__ import annotations
import os
from typing import Dict, Any, List

# ===== CAN driver =====
try:
    from drivers.canbase_merged import UIM342CAN
except Exception as e:
    UIM342CAN = None
    _import_err = e
else:
    _import_err = None

# ===== Config =====
STEPPER_IDS: List[int] = [6, 7, 8]
CAN_CH = os.getenv("CAN_CH", "can0")
CAN_DEBUG = os.getenv("CAN_DEBUG", "0").lower() in ("1", "true", "yes")

# Lazy-open device (satu instance bersama)
_dev: UIM342CAN | None = None
def _get_dev() -> UIM342CAN:
    global _dev
    if _import_err:
        raise RuntimeError(f"drivers.canbase_merged import error: {_import_err}")
    if _dev is None:
        _dev = UIM342CAN(channel=CAN_CH, debug=CAN_DEBUG)
        print(f"[callbacks] CAN opened @ {CAN_CH} debug={CAN_DEBUG}")
    return _dev

def _select_ids(payload: Dict[str, Any]) -> List[int]:
    """Ambil target IDs dari payload ('ids' / 'motor_selection')."""
    if isinstance(payload.get("ids"), list):
        return [i for i in payload["ids"] if i in STEPPER_IDS]
    sel = str(payload.get("motor_selection") or payload.get("motor") or "all").lower()
    if sel in ("all", "stepper", "stepper_only"):
        return STEPPER_IDS[:]
    return STEPPER_IDS[:]

def _pos_cache(state: Dict[str, Any]) -> Dict[int, int]:
    d = state.get("last_pos")
    if not isinstance(d, dict):
        d = {}
        state["last_pos"] = d
    return d

# ===== System handlers =====
def wake_up(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """MO ON untuk ID terpilih."""
    ids = _select_ids(msg); dev = _get_dev()
    for nid in ids:
        try:
            dev.mo(nid, True)
            print(f"[wake_up] MO on @ ID {nid}")
        except Exception as e:
            print(f"[wake_up] ID {nid} ERROR: {e}")
    state["motor_on"] = True

def shutdown(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """MO OFF untuk ID terpilih."""
    ids = _select_ids(msg); dev = _get_dev()
    for nid in ids:
        try:
            dev.mo(nid, False)
            print(f"[shutdown] MO off @ ID {nid}")
        except Exception as e:
            print(f"[shutdown] ID {nid} ERROR: {e}")
    state["motor_on"] = False

def stop(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """STOP (SD decel)."""
    ids = _select_ids(msg); dev = _get_dev()
    for nid in ids:
        try:
            dev.stop(nid)
            print(f"[stop] SD @ ID {nid}")
        except Exception as e:
            print(f"[stop] ID {nid} ERROR: {e}")

def homing(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """Stub homing (belum implement)."""
    print(f"[homing] (stub) ids={_select_ids(msg)}")

def dancing(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """Stub dancing (belum implement)."""
    print(f"[dancing] (stub) ids={_select_ids(msg)}")

def quit(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    state["running"] = False
    print("[quit] stopping main loop")

# ===== Motion stubs (PP/PVT) =====
def pp_joint(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[pp_joint] (stub) joints={msg.get('joints')} time={msg.get('time')} ids={_select_ids(msg)}")

def pp_coor(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[pp_coor] (stub) coor={msg.get('coor')} time={msg.get('time')} ids={_select_ids(msg)}")

def pvt_joint(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[pvt_joint] (stub) joints={msg.get('joints')} time={msg.get('time')} ids={_select_ids(msg)}")

def pvt_coor(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[pvt_coor] (stub) coor={msg.get('coor')} time={msg.get('time')} ids={_select_ids(msg)}")

# ===== Sensors / position =====
def read_position(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """
    Baca absolute position via PA-GET (paling valid di unit kamu).
    Update state['last_pos'][id].
    """
    ids = _select_ids(msg); dev = _get_dev(); cache = _pos_cache(state)
    for nid in ids:
        try:
            pos = dev.read_position(nid, via="pa")
            cache[nid] = pos
            print(f"[read_position] ID {nid} -> {pos}")
        except Exception as e:
            print(f"[read_position] ID {nid} ERROR: {e}")

def read_encoder(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """Alias ke read_position via PA (encoder count absolut)."""
    ids = _select_ids(msg); dev = _get_dev(); cache = _pos_cache(state)
    for nid in ids:
        try:
            pos = dev.read_encoder(nid, via="pa")
            cache[nid] = pos
            print(f"[get_encoder] ID {nid} -> {pos}")
        except Exception as e:
            print(f"[get_encoder] ID {nid} ERROR: {e}")

def set_origin(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """Stub set origin (device-level belum diimplement; pakai software offset nanti)."""
    print(f"[set_origin] (stub) ids={_select_ids(msg)}")

# ===== Export mapping =====
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
