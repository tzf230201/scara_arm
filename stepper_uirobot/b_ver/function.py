from stepper import *

STEPPER_GROUP_ID = 10
STEPPER_MICROSTEP = 128
STEPPER_CPR = 200 * STEPPER_MICROSTEP  # 200 step/rev x 128 microstep = 25600 pulse/rev

# Satuan: degree/pulse dan pulse/degree
STEPPER_DEG_PER_PULSE = 360.0 / STEPPER_CPR   # (ex: 0.0140625 deg/pulse)
STEPPER_PULSE_PER_DEG = STEPPER_CPR / 360.0   # (ex: 71.111... pulse/deg)

def stepper_pulse_to_deg(pulse):
    return float(pulse) * STEPPER_DEG_PER_PULSE

def stepper_deg_to_pulse(deg):
    return int(round(float(deg) * STEPPER_PULSE_PER_DEG))

def stepper_get_all_enc():
    pa2 = stepper_get_pa(6)
    pa3 = stepper_get_pa(7)
    pa4 = stepper_get_pa(8)
    return pa2, pa3, pa4

def stepper_get_all_angle():
    pa2, pa3, pa4 = stepper_get_all_enc()
    pa2_deg = stepper_pulse_to_deg(pa2)
    pa3_deg = stepper_pulse_to_deg(pa3)
    pa4_deg = stepper_pulse_to_deg(pa4)
    return pa2_deg, pa3_deg, pa4_deg

def stepper_set_all_motor_on():
    stepper_set_mo(6, 1)
    stepper_set_mo(7, 1)
    stepper_set_mo(8, 1)

def stepper_set_all_motor_off():
    stepper_set_mo(6, 0)
    stepper_set_mo(7, 0)
    stepper_set_mo(8, 0)

def stepper_set_all_group_id():
    stepper_set_group_id(6, STEPPER_GROUP_ID)
    stepper_set_group_id(7, STEPPER_GROUP_ID)
    stepper_set_group_id(8, STEPPER_GROUP_ID)

def stepper_set_all_unit_ac_dc():
    stepper_set_ac_dc_unit(6, 0)
    stepper_set_ac_dc_unit(7, 0)
    stepper_set_ac_dc_unit(8, 0)

def stepper_set_all_micro_stepping():
    stepper_set_micro_stepping_resolution(6, STEPPER_MICROSTEP)
    stepper_set_micro_stepping_resolution(7, STEPPER_MICROSTEP)
    stepper_set_micro_stepping_resolution(8, STEPPER_MICROSTEP)


def init_stepper():
    stepper_set_all_motor_off()
    ret = stepper_get_ptp_finish_notification(6)
    print(f"PTP Finish Notification (motor 6): {ret}")
    stepper_set_all_group_id()
    stepper_set_all_unit_ac_dc()
    stepper_set_all_micro_stepping()
    
    
    # print(f"validasi")
    
    # group_id = stepper_get_group_id(6)
    # print(f"Group ID for motor 6: {group_id}")

    # unit = stepper_get_ac_dc_unit(6)
    # print(f"AC/DC Unit for motor 6: {unit}")

    # microstep = stepper_get_micro_stepping_resolution(6)
    # print(f"Microstepping for motor 6: {microstep}")

    stepper_set_all_motor_on()