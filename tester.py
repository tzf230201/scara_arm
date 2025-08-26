from stepper import *
import time


# BITRATE_MAP = {
#     0: "1000Kbps",
#     1: "800Kbps",
#     2: "500Kbps",
#     3: "250Kbps",
#     4: "125Kbps"
# }

# br = stepper_get_bitrate(6)
# if br is not None:
#     print(f"Bitrate Stepper 6: {BITRATE_MAP.get(br, 'Unknown')}")
# else:
#     print("Gagal baca bitrate")
    
# node_id = stepper_get_node_id(6)
# if node_id is not None:
#     print(f"Node ID Stepper 6: {node_id}")
# else:
#     print("Gagal baca Node ID")

# group_id = stepper_get_group_id(6)
# if group_id is not None:
#     print(f"Group ID Stepper 6: {group_id}")
# else:
#     print("Gagal baca Group ID")

# # result = stepper_set_ac_dc_unit(6, 1)
# # if result is not None:
# #     print(f"AC/DC Unit sekarang: {result} ({'ms' if result == 1 else 'pulses/sec²'})")
# # else:
# #     print("Gagal set AC/DC Unit")
    
# unit = stepper_get_ac_dc_unit(6)
# print(f"AC/DC Unit (IC[4]): {unit} ({'unit pulses/sec²' if unit == 0 else 'unit ms' if unit == 1 else 'unknown'})")

# # set_cl = stepper_set_using_close_loop(6, True)
# # print(f"Set Closed-loop: {set_cl}")

# get_cl = stepper_get_using_close_loop(6)
# print(f"Closed-loop: {get_cl} ({'using Closed loop' if get_cl == 1 else 'using Open loop' if get_cl == 0 else 'unknown'})")

# # # Enable PTP positioning finish notification
# # ret = stepper_set_ptp_finish_notification(6, 1)
# # print(f"PTP finish notification set to: {ret}")

# # Get PTP positioning finish notification status
# stat = stepper_get_ptp_finish_notification(6)
# print(f"PTP finish notification status: {stat} ({'ENABLED' if stat == 1 else 'DISABLED' if stat == 0 else 'Unknown'})")


# ERROR_CODE_DESC = {
#     0x32: "Instruction Syntax error",
#     0x33: "Instruction Data error",
#     0x34: "Instruction Sub-Index error",
#     0x3C: "SD value is less than DC value",
#     0x3D: "Instruction not allowed when motor is running",
#     0x3E: "BG not allowed when motor driver is OFF",
#     0x3F: "BG not allowed during emergency stopping",
#     0x41: "OG not allowed when motor is running",
# }

# # if stepper_clear_error_report(6):
# #     print("Error report berhasil di-clear!")
# # else:
# #     print("Gagal clear error report.")

# errinfo = stepper_get_error_report(6)

# if errinfo is not None:
#     code = errinfo["error_code"]
#     cw   = errinfo["cw_related"]
#     idx  = errinfo["sub_index"]
#     desc = ERROR_CODE_DESC.get(code, f"Unknown error code 0x{code:02X}")

#     print(f"[Stepper Error Report]")
#     print(f"- Error Code : 0x{code:02X} ({desc})")
#     print(f"- Related CW : 0x{cw:02X}")
#     print(f"- Sub-Index  : 0x{idx:02X}")
# else:
#     print("Tidak ada error (atau gagal baca error report).")

# # print("Set Micro-stepping:", stepper_set_micro_stepping_resolution(6, 16))
# print("Micro-stepping:", stepper_get_micro_stepping_resolution(6))

# # print("Set current:", stepper_set_working_current(6, 2.5))
# print("Working current (A):", stepper_get_working_current(6))

# # print("Set idle %:", stepper_set_percentage_idle_current(6, 50))
# print("Idle %:", stepper_get_percentage_idle_current(6))

# mo = stepper_get_mo(6)
# if mo is not None:
#     print(f"Status Motor Stepper 6: {'ON' if mo == 1 else 'OFF'}")
# else:
#     print("Gagal baca status motor")


# ac = stepper_set_acceleration(6, 1000)
# if ac is not None:
#     print(f"Set acceleration berhasil: {ac}")
# else:
#     print("Gagal set acceleration")


# acc = stepper_get_acceleration(6)
# if acc is not None:
#     print(f"Acceleration sekarang: {acc}")
# else:
#     print("Gagal membaca acceleration")
    
# decel = stepper_set_deceleration(6, 1000)
# if decel is not None:
#     print(f"Set deceleration: {decel}")
# else:
#     print("Gagal set deceleration")

# cur = stepper_get_deceleration(6)
# print(f"Deceleration sekarang: {cur}")

# speed = stepper_set_cut_in_speed(6, 1001)
# if speed is not None:
#     print(f"Set cut-in speed: {speed} pulse/sec^2")
# else:
#     print("Gagal set cut-in speed")

# current = stepper_get_cut_in_speed(6)
# print(f"Cut-in speed sekarang: {current}")

# # Set JV ke -1000
# jv = stepper_set_jv(6, -1000)
# print("JV set:", jv)

# # stepper_begin_motion(6)


# # time.sleep(2)
# # Get current JV
# jv_now = stepper_get_jv(6)
# print("JV now:", jv_now)



# # # Get current JV
# # jv_now = stepper_get_jv(6)
# # print("JV now:", jv_now)

# stepper_stop_motion(6)


# sp_set = stepper_set_sp(6, 1000)
# print("SP set:", sp_set)

# sp_now = stepper_get_sp(6)
# print("SP now:", sp_now)

# pr_set = stepper_set_pr(6, 2000)
# print("PR set:", pr_set)

# pr_now = stepper_get_pr(6)
# print("PR now:", pr_now)

# pa_set = stepper_set_pa(6, 1500)
# print("PA set:", pa_set)

# pa_now = stepper_get_pa(6)
# print("PA now:", pa_now)

# # if stepper_set_origin(6):
# #     print("Origin set berhasil!")
# # else:
# #     print("Gagal set origin.")

# # Set backlash compensation ke 1000 pulse
# # ok = stepper_set_bl(6, 1000)
# # print("Set BL:", "OK" if ok else "Gagal")

# # Get backlash compensation
# bl = stepper_get_bl(6)
# print("Backlash compensation:", bl)


# # # Set JV ke -1000
# # jv = stepper_set_jv(6, -1000)
# # print("JV set:", jv)

# # stepper_begin_motion(6)


# # time.sleep(2)

# def decode_sf_d1(d1):
#     return {
#         "Mode":      "PTP" if d1 & 0b11 else "JOG", # bit0, bit1
#         "SON":       bool((d1 >> 2) & 1),           # bit2
#         "IN1":       bool((d1 >> 3) & 1),           # bit3
#         "IN2":       bool((d1 >> 4) & 1),           # bit4
#         "IN3":       bool((d1 >> 5) & 1),           # bit5
#         "OP1":       bool((d1 >> 6) & 1),           # bit6
#     }

# def decode_sf_d2(d2):
#     return {
#         "STOP":  bool(d2 & (1 << 0)),
#         "PAIF":  bool(d2 & (1 << 1)),
#         "PSIF":  bool(d2 & (1 << 2)),
#         "TLIF":  bool(d2 & (1 << 3)),
#         "LOCK":  bool(d2 & (1 << 5)),
#         "ERR":   bool(d2 & (1 << 7)),
#     }
    
# res = stepper_get_sf_pr(6)
# if res:
#     io_mode = decode_sf_d1(res["d1"])
#     flags   = decode_sf_d2(res["d2"])
#     print(f"== IO & Mode ==")
#     for k, v in io_mode.items():
#         print(f"  {k}: {'ON' if v else 'OFF'}")
#     print(f"== Status Flag ==")
#     for k, v in flags.items():
#         print(f"  {k}: {'YES' if v else 'NO'}")
#     print(f"Current relative position: {res['rel_pos']} pulses")
# else:
#     print("Gagal baca status flags/relative position")

# res2 = stepper_get_sp_pa(6)
# if res2:
#     print(f"Current speed: {res2['speed']} pulses/sec, Absolute position: {res2['abs_pos']} pulses")
# else:
#     print("Gagal baca speed/abs pos")

# if stepper_clear_sf(6):
#     print("Status flags berhasil di-clear!")
# else:
#     print("Gagal clear status flags")
    
    

# # Get current JV
# # jv_now = stepper_get_jv(6)
# # print("JV now:", jv_now)
# # stepper_stop_motion(6)

# print("Motion mode:", stepper_get_motion_mode(6))      # 0=JOG, 1=PTP
# print("Desired motor current (A):", stepper_get_motor_current(6))
# print("Desired speed (pulses/sec):", stepper_get_desired_sp(6))
# print("Desired relative pos (pulses):", stepper_get_desired_pr(6))
# print("Desired absolute pos (pulses):", stepper_get_desired_pa(6))


# # sukses = stepper_pvt_clear_queue(6)
# # print("PVT Table cleared:", sukses)   # True jika sukses
# # # Set first valid row ke 2 (misal) untuk Node ID 6
# # sukses = stepper_pvt_set_first_valid_row(6, 2)
# # print("Set first valid row:", sukses)
# # # Set last valid row ke 32 untuk Node ID 6
# # sukses = stepper_pvt_set_last_valid_row(6, 32)
# # print("Set last valid row:", sukses)

# # # Set management mode ke FIFO (0)
# # sukses = stepper_pvt_set_management_mode(6, 0)
# # print("Set management mode:", sukses)

# # sukses = stepper_pvt_set_pt_time(6, 0)
# # print("Set PT time:", sukses)

# # # Set alert jika queue tinggal 4
# # sukses = stepper_pvt_set_queue_low(6, 4)
# # print("Set queue low value:", sukses)

# # # Set index row berikutnya ke 5
# # sukses = stepper_pvt_set_next_available_writing_row(6, 5)
# # print("Set next writing row:", sukses)

# node_id = 6

# queue_level = stepper_pvt_get_queue(node_id)
# print(f"Queue Level: {queue_level}")

# first_valid_row = stepper_pvt_get_first_valid_row(node_id)
# print(f"First Valid Row: {first_valid_row}")

# last_valid_row = stepper_pvt_get_last_valid_row(node_id)
# print(f"Last Valid Row: {last_valid_row}")

# management_mode = stepper_pvt_get_management_mode(node_id)
# print(f"Management Mode: {management_mode}  # 0=FIFO, 1=Single, 2=Loop")

# pt_time_mode = stepper_pvt_get_pt_time(node_id)
# print(f"PT Time Mode: {pt_time_mode}  # 0=PVT motion")

# queue_low_alert = stepper_pvt_get_queue_low(node_id)
# print(f"Queue Low Alert Value: {queue_low_alert}")

# next_writing_row = stepper_pvt_get_next_available_writing_row(node_id)
# print(f"Next Available Writing Row: {next_writing_row}")


# node = 6
# row = 5

# # Set posisi row 5 ke -1000 pulse
# ok = stepper_pvt_set_position_row_n(node, row, -1000)
# print("Set QP[5] -1000:", ok)

    
# import struct
# d = [37, 0, 24, 252, 255, 255, 0, 0]
# print(struct.unpack('<i', bytes(d[2:6])))   # Output: (-1000,)

# for n in range(8):
#     qp = stepper_pvt_get_position_row_n(6, n)
#     print(f"QP[{n}]: {qp}")
    
# # Set velocity row 6 ke +3 p/s
# success = stepper_pvt_set_velocity_row_n(6, 6, 3)
# print(f"Set QV[6]=3: {success}")

# # Get velocity row 5
# vel = stepper_pvt_get_velocity_row_n(6, 0)
# print(f"QV05]: {vel} pulses/sec")


# # SET QT[5] = 100 ms
# ok = stepper_pvt_set_time_row_n(6, 5, 100)
# print(f"Set QT[5]=100 ms: {ok}")

# # GET QT[5]
# qt5 = stepper_pvt_get_time_row_n(6, 5)
# print(f"QT[5]: {qt5} ms")

# qt5 = stepper_pvt_get_time_row_n(6, 0)
# print(f"QT[5]: {qt5} ms")
# stepper_set_mo(6, False)  # ON dulu

sukses = stepper_pvt_clear_queue(6)
print("PVT Table cleared:", sukses)   # True jika sukses

queue_level = stepper_pvt_get_queue(6)
print(f"Queue Level: {queue_level}")


# stepper_pvt_set_quick_feeding(6, qp=10000, qv=-1000, qt=50)
# # for n in range(8):
# qt, qv, qp = stepper_pvt_get_quick_feeding_row_n(6, 26)
# print(f"QT={qt} ms, QV={qv} pulses/s, QP={qp} pulses")

# # qp = stepper_pvt_get_position_row_n(6, 10)
# print("QP[10]:", qp)]

# print(f"validasi")

# for n in range(50):
#     qp = stepper_pvt_get_position_row_n(6, n)
#     qv = stepper_pvt_get_velocity_row_n(6, n)
#     qt = stepper_pvt_get_time_row_n(6, n)
#     print(f"Row {n}: QT={qt} ms, QV={qv} pulses/s, QP={qp} pulses")

# Set position row 266 ke 100000 pulses

# print(f"v = [0, 0, 50, 0] hasilnya adalah:")
# v = [0, 0, 50, 0]
# print(int.from_bytes(v, byteorder='little', signed=True))  # Output: 3276800

# ok = stepper_pvt_set_pt_data_row_n(6, 266, 100000)
# print(f"Set PT row 266 = 100000: {ok}")

# # Get position row 266
# pos = stepper_pvt_get_pt_data_row_n(6, 266)
# print(f"PT row 266 position: {pos}")
