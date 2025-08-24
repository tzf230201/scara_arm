# callbacks.py — ZMQ worker callbacks (single file)
# - read_position -> degrees (°)
# - read_encoder / get_encoder -> raw pulses (int)
# - set_origin -> software zero per-ID (optional persistence)
from __future__ import annotations
import os, json
from pathlib import Path
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
CAN_CH   = os.getenv("CAN_CH", "can0")
CAN_DEBUG = os.getenv("CAN_DEBUG", "0").lower() in ("1", "true", "yes")

# Scale for degrees
DEFAULT_CPR   = int(os.getenv("CPR", "3200"))      # counts per revolution
DEFAULT_GEAR  = float(os.getenv("GEAR", "1.0"))    # total gear ratio (output:input)
DEFAULT_INV   = int(os.getenv("INV", "0"))         # 1 to invert sign

# Origin persistence
ORIGIN_FILE   = Path(os.path.expanduser(os.getenv("ORIGIN_FILE", "~/.uim342_origin.json")))
SAVE_ORIGIN   = os.getenv("SAVE_ORIGIN", "0").lower() in ("1","true","yes")

# ===== Helpers: env per-ID =====
def _env_int(name: str, default: int) -> int:
    try: return int(os.getenv(name, str(default)))
    except: return default

def _env_float(name: str, default: float) -> float:
    try: return float(os.getenv(name, str(default)))
    except: return default

def _env_bool01(name: str, default: int) -> int:
    v = os.getenv(name, None)
    if v is None: return default
    return 1 if str(v).lower() in ("1","true","yes","on") else 0

def _cpr_for_id(nid: int) -> int:   return _env_int(f"CPR_{nid}",  DEFAULT_CPR)
def _gear_for_id(nid: int) -> float:return _env_float(f"GEAR_{nid}", DEFAULT_GEAR)
def _inv_for_id(nid: int) -> int:   return _env_bool01(f"INV_{nid}", DEFAULT_INV)

# ===== Lazy-open device =====
_dev: UIM342CAN | None = None
def _get_dev() -> UIM342CAN:
    global _dev
    if _import_err:
        raise RuntimeError(f"drivers.canbase_merged import error: {_import_err}")
    if _dev is None:
        _dev = UIM342CAN(channel=CAN_CH, debug=CAN_DEBUG)
        print(f"[callbacks] CAN opened @ {CAN_CH} debug={CAN_DEBUG}")
    return _dev

# ===== Payload / State utils =====
def _select_ids(payload: Dict[str, Any]) -> List[int]:
    if isinstance(payload.get("ids"), list):
        return [i for i in payload["ids"] if i in STEPPER_IDS]
    sel = str(payload.get("motor_selection") or payload.get("motor") or "all").lower()
    if sel in ("all", "stepper", "stepper_only"): return STEPPER_IDS[:]
    return STEPPER_IDS[:]

def _pos_cache_pulses(state: Dict[str, Any]) -> Dict[int, int]:
    d = state.get("last_pos_pulses")
    if not isinstance(d, dict):
        d = {}; state["last_pos_pulses"] = d
    return d

def _pos_cache_deg(state: Dict[str, Any]) -> Dict[int, float]:
    d = state.get("last_pos_deg")
    if not isinstance(d, dict):
        d = {}; state["last_pos_deg"] = d
    return d

def _origin_map(state: Dict[str, Any]) -> Dict[int, int]:
    d = state.get("origin")
    if not isinstance(d, dict):
        d = {}
        if ORIGIN_FILE.exists():
            try:
                loaded = json.loads(ORIGIN_FILE.read_text())
                for k, v in loaded.items():
                    try: d[int(k)] = int(v)
                    except: pass
                print(f"[origin] loaded from {ORIGIN_FILE}: {d}")
            except Exception as e:
                print(f"[origin] load failed: {e}")
        state["origin"] = d
    return d

def _save_origin(state: Dict[str, Any]) -> None:
    if not SAVE_ORIGIN: return
    d = _origin_map(state)
    try:
        ORIGIN_FILE.parent.mkdir(parents=True, exist_ok=True)
        ORIGIN_FILE.write_text(json.dumps(d, indent=2))
        print(f"[origin] saved to {ORIGIN_FILE}")
    except Exception as e:
        print(f"[origin] save failed: {e}")

# ===== Conversion =====
def pulses_to_deg(nid: int, pulses: int, origin_pulses: int) -> float:
    cpr  = max(1, _cpr_for_id(nid))
    gear = max(1e-9, _gear_for_id(nid))
    inv  = -1.0 if _inv_for_id(nid) else 1.0
    deg_per_pulse = 360.0 / (cpr * gear)
    return inv * (pulses - origin_pulses) * deg_per_pulse

# ============================
# System handlers
# ============================
def wake_up(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    ids = _select_ids(msg); dev = _get_dev()
    for nid in ids:
        try:
            dev.mo(nid, True); print(f"[wake_up] MO on @ ID {nid}")
        except Exception as e:
            print(f"[wake_up] ID {nid} ERROR: {e}")
    state["motor_on"] = True

def shutdown(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    ids = _select_ids(msg); dev = _get_dev()
    for nid in ids:
        try:
            dev.mo(nid, False); print(f"[shutdown] MO off @ ID {nid}")
        except Exception as e:
            print(f"[shutdown] ID {nid} ERROR: {e}")
    state["motor_on"] = False

def stop(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    ids = _select_ids(msg); dev = _get_dev()
    for nid in ids:
        try:
            dev.stop(nid); print(f"[stop] SD @ ID {nid}")
        except Exception as e:
            print(f"[stop] ID {nid} ERROR: {e}")

def homing(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[homing] (stub) ids={_select_ids(msg)}")

def dancing(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[dancing] (stub) ids={_select_ids(msg)}")

def quit(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    state["running"] = False; print("[quit] stopping main loop")

# ============================
# Motion stubs (PP/PVT)
# ============================
def pp_joint(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[pp_joint] (stub) joints={msg.get('joints')} time={msg.get('time')} ids={_select_ids(msg)}")

def pp_coor(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[pp_coor] (stub) coor={msg.get('coor')} time={msg.get('time')} ids={_select_ids(msg)}")

def pvt_joint(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[pvt_joint] (stub) joints={msg.get('joints')} time={msg.get('time')} ids={_select_ids(msg)}")

def pvt_coor(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    print(f"[pvt_coor] (stub) coor={msg.get('coor')} time={msg.get('time')} ids={_select_ids(msg)}")

# ============================
# Sensors
# ============================
def read_position(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """
    Return DEGREES: PA-GET pulses -> deg using CPR/GEAR/INV and per-ID origin.
    Updates state['last_pos_pulses'][id] and state['last_pos_deg'][id].
    """
    ids = _select_ids(msg); dev = _get_dev()
    cache_p = _pos_cache_pulses(state)
    cache_d = _pos_cache_deg(state)
    origin  = _origin_map(state)

    for nid in ids:
        try:
            pulses = dev.read_position(nid, via="pa")
            cache_p[nid] = pulses
            deg = pulses_to_deg(nid, pulses, origin.get(nid, 0))
            cache_d[nid] = deg
            print(f"[read_position] ID {nid} -> {deg:.3f}°  (pulses={pulses}, origin={origin.get(nid,0)}, cpr={_cpr_for_id(nid)}, gear={_gear_for_id(nid)})")
        except Exception as e:
            print(f"[read_position] ID {nid} ERROR: {e}")

def read_encoder(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """
    Return RAW PULSES: PA-GET pulses (no conversion).
    Updates state['last_pos_pulses'][id]. Does NOT touch last_pos_deg.
    """
    ids = _select_ids(msg); dev = _get_dev()
    cache_p = _pos_cache_pulses(state)
    for nid in ids:
        try:
            pulses = dev.read_position(nid, via="pa")
            cache_p[nid] = pulses
            print(f"[read_encoder] ID {nid} -> {pulses} pulses")
        except Exception as e:
            print(f"[read_encoder] ID {nid} ERROR: {e}")

def set_origin(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
    """
    Origin:
      - Jika mapping OG hardware tersedia (OG_CW_HEX atau OG_PP_IDX), pakai HW origin.
      - Jika tidak, fallback ke software origin (offset pulses).
    """
    ids = _select_ids(msg); dev = _get_dev()
    origin = _origin_map(state)

    have_hw = bool(os.getenv("OG_CW_HEX") or os.getenv("OG_PP_IDX"))

    for nid in ids:
        try:
            if have_hw:
                dev.hw_origin(nid)
                # setelah HW zero, kita anggap offset software = 0
                origin[nid] = 0
                print(f"[set_origin] (HW OG) done @ ID {nid}")
            else:
                pulses = dev.read_position(nid, via="pa")
                origin[nid] = int(pulses)
                print(f"[set_origin] (SW) ID {nid} origin_pulses={origin[nid]}")
        except Exception as e:
            print(f"[set_origin] ID {nid} ERROR: {e}")

    _save_origin(state)


# ===== Export mapping =====
HANDLERS = {
    # system
    "wake_up":       wake_up,
    "shutdown":      shutdown,
    "stop":          stop,
    "homing":        homing,
    "dancing":       dancing,
    "quit":          quit,
    # motion (stubs)
    "pp_joint":      pp_joint,
    "pp_coor":       pp_coor,
    "pvt_joint":     pvt_joint,
    "pvt_coor":      pvt_coor,
    # sensors
    "read_position": read_position,   # degrees
    "read_encoder":  read_encoder,    # raw pulses
    "get_encoder":   read_encoder,    # alias for compatibility
    "set_origin":    set_origin,
}
