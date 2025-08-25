from stepper import *
import time


BITRATE_MAP = {
    0: "1000Kbps",
    1: "800Kbps",
    2: "500Kbps",
    3: "250Kbps",
    4: "125Kbps"
}

br = stepper_get_bitrate(6)
if br is not None:
    print(f"Bitrate Stepper 6: {BITRATE_MAP.get(br, 'Unknown')}")
else:
    print("Gagal baca bitrate")
    
node_id = stepper_get_node_id(6)
if node_id is not None:
    print(f"Node ID Stepper 6: {node_id}")
else:
    print("Gagal baca Node ID")

group_id = stepper_get_group_id(6)
if group_id is not None:
    print(f"Group ID Stepper 6: {group_id}")
else:
    print("Gagal baca Group ID")

# result = stepper_set_ac_dc_unit(6, 1)
# if result is not None:
#     print(f"AC/DC Unit sekarang: {result} ({'ms' if result == 1 else 'pulses/sec²'})")
# else:
#     print("Gagal set AC/DC Unit")
    
unit = stepper_get_ac_dc_unit(6)
print(f"AC/DC Unit (IC[4]): {unit} ({'unit pulses/sec²' if unit == 0 else 'unit ms' if unit == 1 else 'unknown'})")

# set_cl = stepper_set_using_close_loop(6, True)
# print(f"Set Closed-loop: {set_cl}")

get_cl = stepper_get_using_close_loop(6)
print(f"Closed-loop: {get_cl} ({'using Closed loop' if get_cl == 1 else 'using Open loop' if get_cl == 0 else 'unknown'})")

# # Enable PTP positioning finish notification
# ret = stepper_set_ptp_finish_notification(6, 1)
# print(f"PTP finish notification set to: {ret}")

# Get PTP positioning finish notification status
stat = stepper_get_ptp_finish_notification(6)
print(f"PTP finish notification status: {stat} ({'ENABLED' if stat == 1 else 'DISABLED' if stat == 0 else 'Unknown'})")


ERROR_CODE_DESC = {
    0x32: "Instruction Syntax error",
    0x33: "Instruction Data error",
    0x34: "Instruction Sub-Index error",
    0x3C: "SD value is less than DC value",
    0x3D: "Instruction not allowed when motor is running",
    0x3E: "BG not allowed when motor driver is OFF",
    0x3F: "BG not allowed during emergency stopping",
    0x41: "OG not allowed when motor is running",
}

# if stepper_clear_error_report(6):
#     print("Error report berhasil di-clear!")
# else:
#     print("Gagal clear error report.")

errinfo = stepper_get_error_report(6)

if errinfo is not None:
    code = errinfo["error_code"]
    cw   = errinfo["cw_related"]
    idx  = errinfo["sub_index"]
    desc = ERROR_CODE_DESC.get(code, f"Unknown error code 0x{code:02X}")

    print(f"[Stepper Error Report]")
    print(f"- Error Code : 0x{code:02X} ({desc})")
    print(f"- Related CW : 0x{cw:02X}")
    print(f"- Sub-Index  : 0x{idx:02X}")
else:
    print("Tidak ada error (atau gagal baca error report).")

# print("Set Micro-stepping:", stepper_set_micro_stepping_resolution(6, 16))
print("Micro-stepping:", stepper_get_micro_stepping_resolution(6))

# print("Set current:", stepper_set_working_current(6, 2.5))
print("Working current (A):", stepper_get_working_current(6))

# print("Set idle %:", stepper_set_percentage_idle_current(6, 50))
print("Idle %:", stepper_get_percentage_idle_current(6))

mo = stepper_get_mo(6)
if mo is not None:
    print(f"Status Motor Stepper 6: {'ON' if mo == 1 else 'OFF'}")
else:
    print("Gagal baca status motor")


ac = stepper_set_acceleration(6, 1000)
if ac is not None:
    print(f"Set acceleration berhasil: {ac}")
else:
    print("Gagal set acceleration")


acc = stepper_get_acceleration(6)
if acc is not None:
    print(f"Acceleration sekarang: {acc}")
else:
    print("Gagal membaca acceleration")
    
decel = stepper_set_deceleration(6, 1000)
if decel is not None:
    print(f"Set deceleration: {decel}")
else:
    print("Gagal set deceleration")

cur = stepper_get_deceleration(6)
print(f"Deceleration sekarang: {cur}")

speed = stepper_set_cut_in_speed(6, 1001)
if speed is not None:
    print(f"Set cut-in speed: {speed} pulse/sec^2")
else:
    print("Gagal set cut-in speed")

current = stepper_get_cut_in_speed(6)
print(f"Cut-in speed sekarang: {current}")

# Set JV ke -1000
jv = stepper_set_jv(6, -1000)
print("JV set:", jv)

stepper_begin_motion(6)

# Get current JV
jv_now = stepper_get_jv(6)
print("JV now:", jv_now)

# time.sleep(2)


# # Get current JV
# jv_now = stepper_get_jv(6)
# print("JV now:", jv_now)

stepper_stop_motion(6)

