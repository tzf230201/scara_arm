# # callbacks.py — ZMQ worker callbacks (single file)
# # Works with: main.py (router) + drivers/canbase_merged.py (UIM342CAN)
# # - pp_joint: synchronous start, triangular/trapezoidal profile with caps
# # - read_position: degrees (°) using CPR/GEAR/INV + software-origin offset
# # - read_encoder/get_encoder: raw pulses
# # - set_origin: hardware OG if mapped (env), else software offset (persist optional)

# from __future__ import annotations
# import os, math, json
# from pathlib import Path
# from typing import Dict, Any, List

# # ===== CAN driver =====
# try:
#     from drivers.canbase_merged import UIM342CAN
# except Exception as e:
#     UIM342CAN = None
#     _import_err = e
# else:
#     _import_err = None

# # ===== Config =====
# STEPPER_IDS: List[int] = [6, 7, 8]
# CAN_CH     = os.getenv("CAN_CH", "can0")
# CAN_DEBUG  = os.getenv("CAN_DEBUG", "0").lower() in ("1", "true", "yes")

# # Degree scaling
# DEFAULT_CPR  = int(os.getenv("CPR", "3200"))      # counts per revolution
# DEFAULT_GEAR = float(os.getenv("GEAR", "1.0"))    # total gear ratio (output:input)
# DEFAULT_INV  = int(os.getenv("INV", "0"))         # 1 to invert

# # Motion caps (units already pure pulses/s and pulses/s^2)
# SP_MIN_PPS = int(os.getenv("SP_MIN_PPS", "50"))
# SP_MAX_PPS = int(os.getenv("SP_MAX_PPS", "200000"))   # adjust to device
# AC_MAX     = int(os.getenv("AC_MAX", "1000000"))      # pulses/s^2
# DC_MAX     = int(os.getenv("DC_MAX", str(AC_MAX)))    # pulses/s^2

# # Optional smoothing off (close to triangular)
# SS_ZERO = os.getenv("SS_ZERO", "0").lower() in ("1","true","yes")

# # Origin persistence for software offset
# ORIGIN_FILE = Path(os.path.expanduser(os.getenv("ORIGIN_FILE", "~/.uim342_origin.json")))
# SAVE_ORIGIN = os.getenv("SAVE_ORIGIN", "0").lower() in ("1","true","yes")

# # ===== Helpers: env per-ID =====
# def _env_int(name: str, default: int) -> int:
#     try: return int(os.getenv(name, str(default)))
#     except: return default

# def _env_float(name: str, default: float) -> float:
#     try: return float(os.getenv(name, str(default)))
#     except: return default

# def _env_bool01(name: str, default: int) -> int:
#     v = os.getenv(name, None)
#     if v is None: return default
#     return 1 if str(v).lower() in ("1","true","yes","on") else 0

# def _cpr_for_id(nid: int) -> int:    return _env_int(f"CPR_{nid}",  DEFAULT_CPR)
# def _gear_for_id(nid: int) -> float: return _env_float(f"GEAR_{nid}", DEFAULT_GEAR)
# def _inv_for_id(nid: int) -> int:    return _env_bool01(f"INV_{nid}", DEFAULT_INV)

# # ===== Lazy-open device =====
# _dev: UIM342CAN | None = None
# def _get_dev() -> UIM342CAN:
#     global _dev
#     if _import_err:
#         raise RuntimeError(f"drivers.canbase_merged import error: {_import_err}")
#     if _dev is None:
#         _dev = UIM342CAN(channel=CAN_CH, debug=CAN_DEBUG)
#         print(f"[callbacks] CAN opened @ {CAN_CH} debug={CAN_DEBUG}")
#     return _dev

# # ===== Payload / State utils =====
# def _select_ids(payload: Dict[str, Any]) -> List[int]:
#     """Use msg['ids'] if provided; else motor_selection; default: all 6/7/8."""
#     if isinstance(payload.get("ids"), list):
#         return [i for i in payload["ids"] if i in STEPPER_IDS]
#     sel = str(payload.get("motor_selection") or payload.get("motor") or "all").lower()
#     if sel in ("all", "stepper", "stepper_only"): return STEPPER_IDS[:]
#     return STEPPER_IDS[:]

# def _pos_cache_pulses(state: Dict[str, Any]) -> Dict[int, int]:
#     d = state.get("last_pos_pulses")
#     if not isinstance(d, dict):
#         d = {}; state["last_pos_pulses"] = d
#     return d

# def _pos_cache_deg(state: Dict[str, Any]) -> Dict[int, float]:
#     d = state.get("last_pos_deg")
#     if not isinstance(d, dict):
#         d = {}; state["last_pos_deg"] = d
#     return d

# def _origin_map(state: Dict[str, Any]) -> Dict[int, int]:
#     """Software origin (offset pulses) per-ID, optionally loaded/saved to file."""
#     d = state.get("origin")
#     if not isinstance(d, dict):
#         d = {}
#         if ORIGIN_FILE.exists():
#             try:
#                 loaded = json.loads(ORIGIN_FILE.read_text())
#                 for k, v in loaded.items():
#                     try: d[int(k)] = int(v)
#                     except: pass
#                 print(f"[origin] loaded from {ORIGIN_FILE}: {d}")
#             except Exception as e:
#                 print(f"[origin] load failed: {e}")
#         state["origin"] = d
#     return d

# def _save_origin(state: Dict[str, Any]) -> None:
#     if not SAVE_ORIGIN: return
#     d = _origin_map(state)
#     try:
#         ORIGIN_FILE.parent.mkdir(parents=True, exist_ok=True)
#         ORIGIN_FILE.write_text(json.dumps(d, indent=2))
#         print(f"[origin] saved to {ORIGIN_FILE}")
#     except Exception as e:
#         print(f"[origin] save failed: {e}")

# # ===== Conversions =====
# def pulses_to_deg(nid: int, pulses: int, origin_pulses: int) -> float:
#     cpr  = max(1, _cpr_for_id(nid))
#     gear = max(1e-9, _gear_for_id(nid))
#     inv  = -1.0 if _inv_for_id(nid) else 1.0
#     deg_per_pulse = 360.0 / (cpr * gear)
#     return inv * (pulses - origin_pulses) * deg_per_pulse

# def deg_to_pulses(nid: int, deg: float, origin_pulses: int) -> int:
#     cpr  = max(1, _cpr_for_id(nid))
#     gear = max(1e-9, _gear_for_id(nid))
#     inv  = -1.0 if _inv_for_id(nid) else 1.0
#     pulses_per_deg = (cpr * gear) / 360.0
#     return int(round(origin_pulses + inv * deg * pulses_per_deg))

# # ===== Profile chooser (triangular/trapezoidal with caps) =====
# def _choose_profile_with_caps(dist_pulses: int, time_ms: int) -> tuple[int, int, float]:
#     """
#     From distance D and target time T, choose (SP, AC/DC) with caps.
#     Returns (sp_pps, acc_pps2, T_pred_s) in pure units.

#       Ideal symmetric triangle: v = 2D/T ; a = 4D/T^2
#       Apply caps: SP_MIN/MAX, AC/DC <= AC_MAX/DC_MAX.
#       Predict actual time with chosen caps (triangle vs trapezoid).
#     """
#     D = abs(int(dist_pulses))
#     T = max(1e-3, (time_ms or 0) / 1000.0)  # seconds

#     v_req = 2.0 * D / T            # pps
#     a_req = 4.0 * D / (T * T)      # pps^2

#     sp_pps = int(max(SP_MIN_PPS, min(SP_MAX_PPS, math.ceil(v_req))))
#     acc    = int(min(AC_MAX, max(1, math.ceil(a_req))))
#     dec    = int(min(DC_MAX, max(1, math.ceil(a_req))))  # currently symmetric

#     # Predict time using effective accel (acc)
#     a_eff = float(acc)
#     D_tri_threshold = (sp_pps * sp_pps) / a_eff

#     if D <= D_tri_threshold:
#         # Triangular
#         T_pred = 2.0 * math.sqrt(D / a_eff)
#     else:
#         # Trapezoidal
#         t_acc  = sp_pps / a_eff
#         d_acc  = 0.5 * a_eff * (t_acc ** 2)
#         d_dec  = d_acc
#         d_rem  = max(0.0, D - (d_acc + d_dec))
#         t_cru  = d_rem / max(1.0, sp_pps)
#         T_pred = 2.0 * t_acc + t_cru

#     return sp_pps, acc, T_pred

# # ============================
# # System handlers
# # ============================
# def wake_up(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     ids = _select_ids(msg); dev = _get_dev()
#     for nid in ids:
#         try: dev.mo(nid, True);  print(f"[wake_up] MO on @ ID {nid}")
#         except Exception as e:    print(f"[wake_up] ID {nid} ERROR: {e}")
#     state["motor_on"] = True

# def shutdown(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     ids = _select_ids(msg); dev = _get_dev()
#     for nid in ids:
#         try: dev.mo(nid, False); print(f"[shutdown] MO off @ ID {nid}")
#         except Exception as e:    print(f"[shutdown] ID {nid} ERROR: {e}")
#     state["motor_on"] = False

# def stop(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     ids = _select_ids(msg); dev = _get_dev()
#     for nid in ids:
#         try: dev.stop(nid);       print(f"[stop] SD @ ID {nid}")
#         except Exception as e:    print(f"[stop] ID {nid} ERROR: {e}")

# def homing(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     print(f"[homing] (stub) ids={_select_ids(msg)}")

# def dancing(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     print(f"[dancing] (stub) ids={_select_ids(msg)}")

# def quit(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     state["running"] = False
#     print("[quit] stopping main loop")

# # ============================
# # Motion (PP/PVT)
# # ============================
# def pp_joint(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     """
#     PTP Joint synchronous (J1->ID6, J2->ID7, J3->ID8)
#       - msg['joints']: [deg J1, J2, J3]
#       - msg['time']  : target duration in ms
#     Procedure: MO ON → (optional SS=0) → plan (SP, AC/DC) per axis → stage PA/AC/DC/SP → BG all.
#     """
#     dev = _get_dev()
#     ids_order = [6, 7, 8]
#     joints = msg.get("joints") or []
#     t_ms   = int(msg.get("time") or 0)
#     origin = _origin_map(state)

#     # Ensure motors on
#     for nid in ids_order:
#         try: dev.mo(nid, True)
#         except Exception as e: print(f"[pp_joint] warn: MO on fail @ ID {nid}: {e}")

#     # Optional smoothing off for triangular-like profile
#     if SS_ZERO:
#         for nid in ids_order:
#             try: dev.ss(nid, 0)
#             except Exception as e: print(f"[pp_joint] warn: SS set fail @ ID {nid}: {e}")

#     # Plan per axis
#     plan = []
#     for idx, nid in enumerate(ids_order):
#         if idx >= len(joints): continue
#         try:
#             tgt_deg = float(joints[idx])
#         except Exception:
#             print(f"[pp_joint] skip ID {nid}: invalid deg {joints[idx]!r}")
#             continue

#         try:
#             cur_p = dev.read_position(nid, via="pa")   # current pulses (raw)
#         except Exception as e:
#             print(f"[pp_joint] ID {nid} read pos fail: {e}")
#             continue

#         tgt_p = deg_to_pulses(nid, tgt_deg, origin.get(nid, 0))
#         dist  = tgt_p - cur_p
#         sp_pps, acc_pps2, T_pred = _choose_profile_with_caps(dist, t_ms)

#         plan.append({
#             "nid": nid, "tgt_deg": tgt_deg, "cur": cur_p, "tgt": tgt_p,
#             "dist": dist, "sp": sp_pps, "acc": acc_pps2, "T_pred": T_pred
#         })

#     if not plan:
#         print("[pp_joint] no valid joints to move"); return

#     # Stage PA/AC/DC/SP (no BG)
#     for it in plan:
#         nid = it["nid"]
#         try:
#             dev.pa(nid, it["tgt"])   # absolute pulses
#             dev.ac(nid, it["acc"])   # pulses/s^2
#             dev.dc(nid, it["acc"])   # pulses/s^2
#             dev.sp(nid, it["sp"])    # pulses/s
#         except Exception as e:
#             print(f"[pp_joint] stage fail @ ID {nid}: {e}")

#     # BG all -> start together
#     for it in plan:
#         nid = it["nid"]
#         try: dev.bg(nid)
#         except Exception as e: print(f"[pp_joint] BG fail @ ID {nid}: {e}")

#     # Cache & log
#     for it in plan:
#         nid = it["nid"]
#         _pos_cache_pulses(state)[nid] = it["tgt"]
#         _pos_cache_deg(state)[nid]    = it["tgt_deg"]
#         print(f"[pp_joint] ID {nid}: tgt={it['tgt_deg']:.3f}° → pulses {it['tgt']} "
#               f"(cur {it['cur']}, dist {it['dist']})  SP={it['sp']}pps  AC/DC={it['acc']}pps² "
#               f"| T_req={t_ms/1000:.3f}s  T_pred≈{it['T_pred']:.3f}s "
#               f"[caps: SP≤{SP_MAX_PPS}, AC/DC≤{AC_MAX}]")

# def pp_coor(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     print(f"[pp_coor] (stub) coor={msg.get('coor')} time={msg.get('time')} ids={_select_ids(msg)}")

# def pvt_joint(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     print(f"[pvt_joint] (stub) joints={msg.get('joints')} time={msg.get('time')} ids={_select_ids(msg)}")

# def pvt_coor(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     print(f"[pvt_coor] (stub) coor={msg.get('coor')} time={msg.get('time')} ids={_select_ids(msg)}")

# # ============================
# # Sensors
# # ============================
# def read_position(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     """
#     Return DEGREES: PA-GET pulses -> degrees using CPR/GEAR/INV & software-origin.
#     Updates state['last_pos_pulses'][id] and state['last_pos_deg'][id].
#     """
#     ids = _select_ids(msg); dev = _get_dev()
#     cache_p = _pos_cache_pulses(state)
#     cache_d = _pos_cache_deg(state)
#     origin  = _origin_map(state)

#     for nid in ids:
#         try:
#             pulses = dev.read_position(nid, via="pa")
#             cache_p[nid] = pulses
#             deg = pulses_to_deg(nid, pulses, origin.get(nid, 0))
#             cache_d[nid] = deg
#             print(f"[read_position] ID {nid} -> {deg:.3f}°  (pulses={pulses}, origin={origin.get(nid,0)}, cpr={_cpr_for_id(nid)}, gear={_gear_for_id(nid)})")
#         except Exception as e:
#             print(f"[read_position] ID {nid} ERROR: {e}")

# def read_encoder(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     """
#     Return RAW PULSES: PA-GET pulses (no conversion).
#     Updates state['last_pos_pulses'][id].
#     """
#     ids = _select_ids(msg); dev = _get_dev()
#     cache_p = _pos_cache_pulses(state)
#     for nid in ids:
#         try:
#             pulses = dev.read_position(nid, via="pa")
#             cache_p[nid] = pulses
#             print(f"[read_encoder] ID {nid} -> {pulses} pulses")
#         except Exception as e:
#             print(f"[read_encoder] ID {nid} ERROR: {e}")

# # alias
# get_encoder = read_encoder

# # ============================
# # Origin (hardware OG if available, else SW offset)
# # ============================
# def set_origin(msg: Dict[str, Any], state: Dict[str, Any]) -> None:
#     """
#     If env OG mapping present, use hardware origin (OG).
#       - OG_CW_HEX: special CW hex string (e.g., '0x33')
#       - OG_PP_IDX: PP[index] to write 1 for zero
#     Else, use software origin (store current PA pulses as offset).
#     """
#     ids = _select_ids(msg); dev = _get_dev()
#     origin = _origin_map(state)

#     have_hw = bool(os.getenv("OG_CW_HEX") or os.getenv("OG_PP_IDX"))

#     for nid in ids:
#         try:
#             if have_hw and hasattr(dev, "hw_origin"):
#                 dev.hw_origin(nid)
#                 origin[nid] = 0
#                 print(f"[set_origin] (HW OG) done @ ID {nid}")
#             else:
#                 pulses = dev.read_position(nid, via="pa")
#                 origin[nid] = int(pulses)
#                 print(f"[set_origin] (SW) ID {nid} origin_pulses={origin[nid]}")
#         except Exception as e:
#             print(f"[set_origin] ID {nid} ERROR: {e}")

#     _save_origin(state)

# # ===== Export mapping =====
# HANDLERS = {
#     # system
#     "wake_up":       wake_up,
#     "shutdown":      shutdown,
#     "stop":          stop,
#     "homing":        homing,
#     "dancing":       dancing,
#     "quit":          quit,
#     # motion
#     "pp_joint":      pp_joint,
#     "pp_coor":       pp_coor,
#     "pvt_joint":     pvt_joint,
#     "pvt_coor":      pvt_coor,
#     # sensors
#     "read_position": read_position,   # degrees
#     "read_encoder":  read_encoder,    # pulses
#     "get_encoder":   get_encoder,     # alias
#     # origin
#     "set_origin":    set_origin,
# }
