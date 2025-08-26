from uim342ab import *

def print_yellow(text):
    YELLOW = "\033[93m"
    RESET = "\033[0m"
    print(f"{YELLOW}{text}{RESET}")

node = 6

bitrate = uim342ab_get_bitrate(node)
print_yellow(f"Bitrate saat ini: {bitrate}")

node_id = uim342ab_get_node_id(node)
print_yellow(f"Node ID saat ini: {node_id}")

group_id = uim342ab_get_group_id(node)
print_yellow(f"Group ID saat ini: {group_id}")

ac_dc = uim342ab_get_ac_dc_unit(node)
unit = 'milliseconds' if ac_dc == 1 else 'pulses' if ac_dc == 0 else 'Unknown'
print_yellow(f"AC/DC unit: {ac_dc} ({unit})")

loop = uim342ab_get_using_close_loop(node)
loop_mode = 'Closed-loop' if loop == 1 else 'Open-loop' if loop == 0 else 'Unknown'
print_yellow(f"Closed-loop mode: {loop} ({loop_mode})")


res = uim342ab_set_ptp_finish_notification(6, False)
print_yellow(f"Set IE[8] → {res} ({'enable' if res==1 else 'disable' if res==0 else 'unknown'})")

val = uim342ab_get_ptp_finish_notification(6)
print_yellow(f"PTP finish notify (IE[8]) sekarang: {val} ({'enable' if val==1 else 'disable' if val==0 else 'unknown'})")

# mapping deskripsi error
ERROR_CODE_DESC = {
    0x32: "Instruction syntax error",
    0x33: "Instruction data error",
    0x34: "Instruction sub-index error",
    0x3C: "SD < DC (invalid)",
    0x3D: "Instruction not allowed while motor running",
    0x3E: "BG not allowed when motor driver OFF",
    0x3F: "BG not allowed during emergency stop",
    0x41: "OG not allowed when motor is running",
}

def print_yellow(text: str):
    print(f"\033[93m{text}\033[0m")

NODE = 6

# --- GET newest error ---
errinfo = uim342ab_get_error_report(NODE, 0)
if errinfo is None:
    print_yellow("[✗] Tidak ada error atau gagal baca ER[0].")
else:
    code = errinfo["error_code"]
    cw   = errinfo["cw_related"]
    idx  = errinfo["sub_index"]
    desc = ERROR_CODE_DESC.get(code, f"Unknown error code 0x{code:02X}")
    print_yellow("[Stepper Error Report — Newest]")
    print_yellow(f"- Error Code : 0x{code:02X} ({desc})")
    print_yellow(f"- Related CW : 0x{cw:02X}")
    print_yellow(f"- Sub-Index  : 0x{idx:02X}")

# --- CLEAR all errors ---
ok = uim342ab_clear_error_report(NODE)
print_yellow("[✓] Error report berhasil di-clear!" if ok else "[✗] Gagal clear error report.")

# --- VERIFY again ---
errinfo2 = uim342ab_get_error_report(NODE, 0)
if errinfo2 is None:
    print_yellow("[•] Setelah clear: tidak ada error (atau gagal baca).")
else:
    code = errinfo2["error_code"]
    desc = ERROR_CODE_DESC.get(code, f"Unknown error code 0x{code:02X}")
    print_yellow(f"[•] Setelah clear: masih ada error 0x{code:02X} ({desc})")

