from stepper import *

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
    
mo = stepper_get_mo(6)
if mo is not None:
    print(f"Status Motor Stepper 6: {'ON' if mo == 1 else 'OFF'}")
else:
    print("Gagal baca status motor")
    
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

