# stepper.py

from canbase import simplecan3_write_read
import struct

# List mnemonic dan nilai Control Word (CW)
MNEMONIC = {
    "PP": 0x01,
    "IC": 0x06,
    "IE": 0x07,
    "ML": 0x0B,
    "SN": 0x0C,
    "ER": 0x0F,
    "QE": 0x3D,
    "SY": 0x7E,

    "MT": 0x10,
    "MO": 0x15,
    "BG": 0x16,
    "ST": 0x17,
    "MF": 0x18,

    "AC": 0x19,
    "DC": 0x1A,
    "SS": 0x1B,
    "SD": 0x1C,
    "JV": 0x1D,
    "SP": 0x1E,
    "PR": 0x1F,
    "PA": 0x20,
    "OG": 0x21,
    "BL": 0x2D,
    "MS": 0x11,
    "DV": 0x2E,

    "IL": 0x34,
    "TG": 0x35,
    "DT": 0x36,

    "RT": 0x5A,
    "MP": 0x22,
    "PV": 0x24,
    "QP": 0x25,
    "QV": 0x26,
    "QT": 0x27,
    "QF": 0x29,
    "PT": 0x23,
}
def stepper_set_bitrate(node_id, bitrate_code):
    """
    Set bitrate stepper.
    :param node_id: ID device sekarang
    :param bitrate_code: kode bitrate (0-4)
    :return: bitrate baru jika sukses, None jika gagal
    
    − Within a specific CAN network, Node IDs and Group IDs of all UIM devices should never be overlapped.
    − Protocol Parameters will take effectiveness after reboot the UIM device.
    − The set value will be saved to EEPROM only when MO=0; otherwise, it stays in RAM and is lost after power off.
    """
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 2, [5, bitrate_code])
    if err == 0 and resp["dl"] > 1 and resp["data"][0] == 5:
        return resp["data"][1]
    return None

def stepper_get_bitrate(node_id):
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [5])
    if err == 0 and resp["dl"] > 2 and resp["data"][1] == 5:
        return resp["data"][2]   # 2 = 500Kbps
    return None

def stepper_set_node_id(node_id, new_node_id):
    """
    Set CAN Node ID stepper.
    :param node_id: node lama (ID device sekarang)
    :param new_node_id: node baru yang di-set (1-127)
    :return: node_id baru jika sukses, None jika gagal
    
    − Within a specific CAN network, Node IDs and Group IDs of all UIM devices should never be overlapped.
    − Protocol Parameters will take effectiveness after reboot the UIM device.
    − The set value will be saved to EEPROM only when MO=0; otherwise, it stays in RAM and is lost after power off.
    """
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 2, [7, new_node_id])
    if err == 0 and resp["dl"] > 1 and resp["data"][0] == 7:
        return resp["data"][1]
    return None

def stepper_get_node_id(node_id):
    """
    Get CAN Node ID dari stepper.
    :param node_id: ID device sekarang
    :return: node_id (number) atau None jika gagal
    """
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [7])
    if err == 0 and resp["dl"] > 2 and resp["data"][1] == 7:
        return resp["data"][2]
    return None

def stepper_set_group_id(node_id, new_group_id):
    """
    Set Group ID stepper.
    :param node_id: ID device sekarang
    :param new_group_id: group id baru (1-127)
    :return: group_id baru jika sukses, None jika gagal
    
    − Within a specific CAN network, Node IDs and Group IDs of all UIM devices should never be overlapped.
    − Protocol Parameters will take effectiveness after reboot the UIM device.
    − The set value will be saved to EEPROM only when MO=0; otherwise, it stays in RAM and is lost after power off.
    """
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 2, [8, new_group_id])
    if err == 0 and resp["dl"] > 1 and resp["data"][0] == 8:
        return resp["data"][1]
    return None

def stepper_get_group_id(node_id):
    """
    Get Group ID dari stepper.
    :param node_id: ID device sekarang
    :return: group_id (number) atau None jika gagal
    """
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [8])
    if err == 0 and resp["dl"] > 2 and resp["data"][1] == 8:
        return resp["data"][2]
    return None

def stepper_set_ac_dc_unit(node_id, unit):
    """
    Set Units for AC and DC (IC[4]):
    unit = 0 → pulse/sec²
    unit = 1 → millisecond
    """
    cw = MNEMONIC["IC"]
    # SET: DL=3, data=[4, 0, unit]
    val = 1 if unit else 0
    err, resp = simplecan3_write_read(node_id, cw, 3, [4, 0, val])
    # Balasan: data[0]=4, data[1]=0, data[2]=val
    if err == 0 and resp["dl"] > 2 and resp["data"][0] == 4:
        return resp["data"][2]
    return None

def stepper_get_ac_dc_unit(node_id):
    """
    Get Units for AC and DC (IC[4]): 0 = pulse/sec², 1 = millisecond
    """
    cw = MNEMONIC["IC"]
    # GET: DL=1, data=[4]
    err, resp = simplecan3_write_read(node_id, cw, 1, [4])
    if err == 0 and resp["dl"] > 2 and resp["data"][1] == 4:
        # Data ada di resp["data"][2]: 0=pulse/sec², 1=ms
        return resp["data"][2]
    return None

def stepper_set_using_close_loop(node_id, enable):
    """
    Set Using Closed-loop Control (IC[6]): 0 = Open loop, 1 = Closed loop
    """
    cw = MNEMONIC["IC"]
    # SET: DL=3, data=[6, 0, 1] untuk enable closed loop
    val = 1 if enable else 0
    err, resp = simplecan3_write_read(node_id, cw, 3, [6, 0, val])
    # Balasan: data[0]=6, data[1]=0, data[2]=val
    if err == 0 and resp["dl"] > 2 and resp["data"][0] == 6:
        return resp["data"][2]
    return None

def stepper_get_using_close_loop(node_id):
    """
    Get Using Closed-loop Control (IC[6]): 0 = Open loop, 1 = Closed loop
    """
    cw = MNEMONIC["IC"]
    # GET: DL=1, data=[6]
    err, resp = simplecan3_write_read(node_id, cw, 1, [6])
    # Balasan: data[0]=6, data[1]=0, data[2]=val
    if err == 0 and resp["dl"] > 2 and resp["data"][0] == 6:
        return resp["data"][2]
    return None

def stepper_set_ptp_finish_notification(node_id, enable):
    cw = MNEMONIC["IE"]
    val = 1 if enable else 0
    err, resp = simplecan3_write_read(node_id, cw, 3, [3, 0, val])
    # Balasan: data[0]=CW, data[1]=3, data[2]=val
    if err == 0 and resp["dl"] > 2 and resp["data"][1] == 3:
        return resp["data"][2]
    return None

def stepper_get_ptp_finish_notification(node_id):
    """
    Get PTP positioning finish notification status (IE[3]): 0 = disable, 1 = enable
    """
    cw = MNEMONIC["IE"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [3])
    # Balasan: data[0]=CW, data[1]=3 (index), data[2]=val
    if err == 0 and resp["dl"] > 2 and resp["data"][1] == 3:
        return resp["data"][2]
    return None

def stepper_clear_error_report(node_id):
    """
    Clear All Error Info (ER[0]=0).
    Return True jika sukses, False jika gagal.
    """
    cw = MNEMONIC["ER"]
    # SET: DL=2, data=[0, 0] (index 0, nilai 0)
    err, resp = simplecan3_write_read(node_id, cw, 2, [0, 0])
    # Balasan: data[0]=0 (index), data[1]=0 (no error), data[2]...=0 (reserve)
    if err == 0 and resp["dl"] >= 2 and resp["data"][0] == 0 and resp["data"][1] == 0:
        return True
    return False


def stepper_get_error_report(node_id):
    """
    Get the newest error info (ER[0]).
    Return: dict berisi error code, CW terkait error, sub-index, dll, atau None jika gagal.
    """
    cw = MNEMONIC["ER"]
    # GET: DL=1, data=[0] (index 0 = newest error)
    err, resp = simplecan3_write_read(node_id, cw, 1, [0])
    if err == 0 and resp["dl"] >= 6 and resp["data"][0] == 0:
        return {
            "error_code":   resp["data"][1],   # Data [d1]: error code
            "cw_related":   resp["data"][2],   # Data [d2]: CW related to error
            "sub_index":    resp["data"][3],   # Data [d3]: Sub-index
            "reserve1":     resp["data"][4],   # Data [d4]: Reserve
            "reserve2":     resp["data"][5],   # Data [d5]: Reserve
        }
    return None

def stepper_set_micro_stepping_resolution(node_id, resolution):
    cw = MNEMONIC["MT"]
    val_lo = resolution & 0xFF
    val_hi = (resolution >> 8) & 0xFF
    # SET: data=[0, 0, lo, hi]
    err, resp = simplecan3_write_read(node_id, cw, 3, [0, val_lo, val_hi])
    # Balasan: data[0]=0, data[1]=0, data[2:3]=resolution
    if err == 0 and resp["dl"] > 2 and resp["data"][0] == 0:
        return resp["data"][2] | (resp["data"][3] << 8)
    return None

def stepper_get_micro_stepping_resolution(node_id):
    cw = MNEMONIC["MT"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [0])
    # Balasan: data[0]=CW(16), data[1]=0 (index), data[2:3]=resolution
    if err == 0 and resp["dl"] > 2 and resp["data"][1] == 0:
        return resp["data"][2] | (resp["data"][3] << 8)
    return None


def stepper_set_working_current(node_id, current):
    cw = MNEMONIC["MT"]
    raw = int(round(current * 10))  # Convert Ampere to 0.1A unit (eg. 2.8A -> 28)
    val_lo = raw & 0xFF
    val_hi = (raw >> 8) & 0xFF
    # SET: data=[1, 0, lo, hi]
    err, resp = simplecan3_write_read(node_id, cw, 3, [1, val_lo, val_hi])
    if err == 0 and resp["dl"] > 2 and resp["data"][0] == 1:
        raw = resp["data"][2] | (resp["data"][3] << 8)
        return raw / 10.0
    return None

def stepper_get_working_current(node_id):
    cw = MNEMONIC["MT"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [1])
    # Balasan: data[0]=CW(16), data[1]=1 (index), data[2:3]=current
    if err == 0 and resp["dl"] > 2 and resp["data"][1] == 1:
        raw = resp["data"][2] | (resp["data"][3] << 8)
        return raw / 10.0
    return None

def stepper_set_percentage_idle_current(node_id, percent):
    cw = MNEMONIC["MT"]
    val_lo = percent & 0xFF
    val_hi = (percent >> 8) & 0xFF
    err, resp = simplecan3_write_read(node_id, cw, 3, [2, val_lo, val_hi])
    if err == 0 and resp["dl"] > 2 and resp["data"][0] == 2:
        return resp["data"][2] | (resp["data"][3] << 8)
    return None

def stepper_get_percentage_idle_current(node_id):
    cw = MNEMONIC["MT"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [2])
    # Balasan: data[0]=CW(16), data[1]=2 (index), data[2:3]=percent
    if err == 0 and resp["dl"] > 2 and resp["data"][1] == 2:
        return resp["data"][2] | (resp["data"][3] << 8)
    return None

def stepper_set_mo(node_id, enable: int):
    cw = MNEMONIC["MO"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [1 if enable else 0])
    if err == 0 and resp["dl"] > 0:
        return resp["data"][0]
    return None

def stepper_get_mo(node_id):
    cw = MNEMONIC["MO"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    if err == 0 and resp["dl"] > 1:
        return resp["data"][1]    # data[1] = status (0=OFF, 1=ON)
    return None                   # None jika error/timeout/format aneh

def stepper_begin_motion(node_id):
    """
    Kirim perintah Begin Motion (BG) ke stepper node_id.
    Return True jika ACK diterima, False jika gagal.
    """
    cw = MNEMONIC["BG"]
    # BG tidak perlu data (DL=0, data=[])
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    # Balasan: data[0]=CW(0x16), data[1:4]=don't care
    if err == 0 and resp["dl"] >= 1 and resp["data"][0] == cw:
        return True
    return False

def stepper_stop_motion(node_id):
    """
    Kirim perintah Stop Motion (ST) ke stepper node_id.
    Return True jika ACK diterima, False jika gagal.
    """
    cw = MNEMONIC["ST"]
    # Stop tidak perlu data (DL=0, data=[])
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    # Balasan: data[0]=CW(0x17), tidak ada data lain
    if err == 0 and resp["dl"] >= 1 and resp["data"][0] == cw:
        return True
    return False

def stepper_set_acceleration(node_id, accel):
    """
    Set acceleration (pulse/sec^2).
    accel: int (0...2^32-1)
    Return nilai yang diset jika sukses, None jika gagal.
    """
    cw = MNEMONIC["AC"]
    # Ubah accel jadi 4 byte little-endian
    acc_bytes = [accel & 0xFF, (accel >> 8) & 0xFF, (accel >> 16) & 0xFF, (accel >> 24) & 0xFF]
    # Kirim data=[accel_lsb, accel, accel, accel_msb]
    err, resp = simplecan3_write_read(node_id, cw, 4, acc_bytes)
    # Balasan: data[0]=CW(0x19), data[1:4]=acknowledged value
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        value = (resp["data"][1] |
                 (resp["data"][2] << 8) |
                 (resp["data"][3] << 16) |
                 (resp["data"][4] << 24))
        return value
    return None

def stepper_get_acceleration(node_id):
    """
    Get acceleration (pulse/sec^2) dari stepper.
    Return nilai acceleration (int) jika sukses, None jika gagal.
    """
    cw = MNEMONIC["AC"]
    # GET: DL=0, data=[]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    # Balasan: data[0]=CW(0x19), data[1:4]=acceleration value
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        value = (resp["data"][1] |
                 (resp["data"][2] << 8) |
                 (resp["data"][3] << 16) |
                 (resp["data"][4] << 24))
        return value
    return None

def stepper_set_deceleration(node_id, decel):
    """
    Set deceleration (pulse/sec^2).
    decel: int (0...2^32-1)
    Return nilai yang diset jika sukses, None jika gagal.
    """
    cw = MNEMONIC["DC"]
    decel_bytes = [
        decel & 0xFF,
        (decel >> 8) & 0xFF,
        (decel >> 16) & 0xFF,
        (decel >> 24) & 0xFF
    ]
    err, resp = simplecan3_write_read(node_id, cw, 4, decel_bytes)
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        value = (
            resp["data"][1] |
            (resp["data"][2] << 8) |
            (resp["data"][3] << 16) |
            (resp["data"][4] << 24)
        )
        return value
    return None

def stepper_get_deceleration(node_id):
    """
    Get deceleration (pulse/sec^2) dari stepper.
    Return nilai deceleration (int) jika sukses, None jika gagal.
    """
    cw = MNEMONIC["DC"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        value = (
            resp["data"][1] |
            (resp["data"][2] << 8) |
            (resp["data"][3] << 16) |
            (resp["data"][4] << 24)
        )
        return value
    return None

def stepper_set_cut_in_speed(node_id, speed):
    """
    Set Cut-in Speed (pulse/sec^2).
    speed: int (0...2^32-1)
    Return nilai yang diset jika sukses, None jika gagal.
    """
    cw = MNEMONIC["SS"]
    speed_bytes = [
        speed & 0xFF,
        (speed >> 8) & 0xFF,
        (speed >> 16) & 0xFF,
        (speed >> 24) & 0xFF
    ]
    err, resp = simplecan3_write_read(node_id, cw, 4, speed_bytes)
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        value = (
            resp["data"][1] |
            (resp["data"][2] << 8) |
            (resp["data"][3] << 16) |
            (resp["data"][4] << 24)
        )
        return value
    return None


def stepper_get_cut_in_speed(node_id):
    """
    Get Cut-in Speed (pulse/sec^2) dari stepper.
    Return nilai cut-in speed (int) jika sukses, None jika gagal.
    """
    cw = MNEMONIC["SS"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        value = (
            resp["data"][1] |
            (resp["data"][2] << 8) |
            (resp["data"][3] << 16) |
            (resp["data"][4] << 24)
        )
        return value
    return None

def stepper_set_jv(node_id, vel):
    """
    Set desired Jog Velocity (pulse/sec, signed int32).
    Return nilai JV yang diset jika sukses, None jika gagal.
    """
    cw = MNEMONIC["JV"]
    vel = vel & 0xFFFFFFFF
    vel_bytes = [
        vel & 0xFF,
        (vel >> 8) & 0xFF,
        (vel >> 16) & 0xFF,
        (vel >> 24) & 0xFF
    ]
    err, resp = simplecan3_write_read(node_id, cw, 4, vel_bytes)
    # Balasan: data[0]=0x2E (DV), data[1]=0x02, data[2:6]=JV value
    if err == 0 and resp["dl"] >= 6 and resp["data"][0] == MNEMONIC["DV"] and resp["data"][1] == 2:
        raw = (
            resp["data"][2] |
            (resp["data"][3] << 8) |
            (resp["data"][4] << 16) |
            (resp["data"][5] << 24)
        )
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw
    return None




def stepper_get_jv(node_id):
    """
    Get current Jog Velocity (pulse/sec).
    Return nilai JV (int, bisa negatif) jika sukses, None jika gagal.
    """
    cw = MNEMONIC["JV"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        # 4-byte signed (little-endian)
        raw = (
            resp["data"][1] |
            (resp["data"][2] << 8) |
            (resp["data"][3] << 16) |
            (resp["data"][4] << 24)
        )
        # Signed convert
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw
    return None

def stepper_set_sp(node_id, speed):
    """
    Set desired PTP speed (pulse/sec, signed int32).
    Return nilai SP yang diset jika sukses, None jika gagal.
    """
    cw = MNEMONIC["SP"]
    # 4-byte signed (little-endian)
    speed = speed & 0xFFFFFFFF
    speed_bytes = [
        speed & 0xFF,
        (speed >> 8) & 0xFF,
        (speed >> 16) & 0xFF,
        (speed >> 24) & 0xFF
    ]
    err, resp = simplecan3_write_read(node_id, cw, 4, speed_bytes)
    # Balasan: data[0]=0x2E (DV), data[1]=0x02, data[2:6]=SP value
    if err == 0 and resp["dl"] >= 6 and resp["data"][0] == MNEMONIC["DV"] and resp["data"][1] == 2:
        raw = (
            resp["data"][2] |
            (resp["data"][3] << 8) |
            (resp["data"][4] << 16) |
            (resp["data"][5] << 24)
        )
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw
    return None

def stepper_get_sp(node_id):
    """
    Get current PTP speed (pulse/sec).
    Return nilai SP (int, bisa negatif) jika sukses, None jika gagal.
    """
    cw = MNEMONIC["SP"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        # 4-byte signed (little-endian)
        raw = (
            resp["data"][1] |
            (resp["data"][2] << 8) |
            (resp["data"][3] << 16) |
            (resp["data"][4] << 24)
        )
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw
    return None

def stepper_set_pr(node_id, position):
    """
    Set desired relative position (signed 32-bit, pulse).
    Return nilai PR yang diset jika sukses, None jika gagal.
    """
    cw = MNEMONIC["PR"]
    # 4-byte signed (little-endian)
    position = position & 0xFFFFFFFF
    pos_bytes = [
        position & 0xFF,
        (position >> 8) & 0xFF,
        (position >> 16) & 0xFF,
        (position >> 24) & 0xFF
    ]
    err, resp = simplecan3_write_read(node_id, cw, 4, pos_bytes)
    # Balasan: data[0]=0x2E (DV), data[1]=0x03, data[2:6]=PR value
    if err == 0 and resp["dl"] >= 6 and resp["data"][0] == MNEMONIC["DV"] and resp["data"][1] == 3:
        raw = (
            resp["data"][2] |
            (resp["data"][3] << 8) |
            (resp["data"][4] << 16) |
            (resp["data"][5] << 24)
        )
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw
    return None

def stepper_get_pr(node_id):
    """
    Get current relative position (pulse).
    Return nilai PR (int, bisa negatif) jika sukses, None jika gagal.
    """
    cw = MNEMONIC["PR"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        # 4-byte signed (little-endian)
        raw = (
            resp["data"][1] |
            (resp["data"][2] << 8) |
            (resp["data"][3] << 16) |
            (resp["data"][4] << 24)
        )
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw
    return None


def stepper_set_pa(node_id, position):
    """
    Set desired absolute position (signed 32-bit, pulse).
    Return nilai PA yang diset jika sukses, None jika gagal.
    """
    cw = MNEMONIC["PA"]
    # 4-byte signed (little-endian)
    position = position & 0xFFFFFFFF
    pos_bytes = [
        position & 0xFF,
        (position >> 8) & 0xFF,
        (position >> 16) & 0xFF,
        (position >> 24) & 0xFF
    ]
    err, resp = simplecan3_write_read(node_id, cw, 4, pos_bytes)
    # Balasan: data[0]=0x2E (DV), data[1]=0x04, data[2:6]=PA value
    if err == 0 and resp["dl"] >= 6 and resp["data"][0] == MNEMONIC["DV"] and resp["data"][1] == 4:
        raw = (
            resp["data"][2] |
            (resp["data"][3] << 8) |
            (resp["data"][4] << 16) |
            (resp["data"][5] << 24)
        )
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw
    return None

def stepper_get_pa(node_id):
    """
    Get current absolute position (pulse).
    Return nilai PA (int, bisa negatif) jika sukses, None jika gagal.
    """
    cw = MNEMONIC["PA"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    if err == 0 and resp["dl"] > 4 and resp["data"][0] == cw:
        # 4-byte signed (little-endian)
        raw = (
            resp["data"][1] |
            (resp["data"][2] << 8) |
            (resp["data"][3] << 16) |
            (resp["data"][4] << 24)
        )
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw
    return None

def stepper_set_origin(node_id):
    """
    Set current position as origin (zero) by clearing the position counter.
    Return True jika berhasil, False jika gagal.
    """
    cw = MNEMONIC["OG"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    # ACK: data kosong (dl==0), resp["cw"] harus OG
    if err == 0 and resp["cw"] == cw:
        return True
    return False

def stepper_set_bl(node_id, value):
    """
    Set backlash compensation (BL) dalam satuan pulse (0..65535).
    value: int
    Return True jika sukses, False jika gagal.
    """
    cw = MNEMONIC["BL"]
    # Format data: 4 bytes, [value_lsb, value_msb, 0, 0]
    value = int(value) & 0xFFFF
    data = [value & 0xFF, (value >> 8) & 0xFF, 0, 0]
    err, resp = simplecan3_write_read(node_id, cw, 4, data)
    if err == 0 and resp["dl"] == 4:
        return True
    return False

def stepper_get_bl(node_id):
    """
    Get backlash compensation value (BL) dalam satuan pulse.
    Return int value, atau None jika gagal.
    """
    cw = MNEMONIC["BL"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    if err == 0 and len(resp["data"]) >= 3:
        value = resp["data"][1] | (resp["data"][2] << 8)
        return value
    return None

def stepper_get_sf_pr(node_id):
    cw = MNEMONIC["MS"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [0])
    if err == 0 and resp["dl"] >= 8:
        d1 = resp["data"][2]
        d2 = resp["data"][3]
        pr_bytes = resp["data"][4:8]
        pr = int.from_bytes(pr_bytes, byteorder='little', signed=True)
        return {"d1": d1, "d2": d2, "rel_pos": pr}
    return None

def stepper_get_sp_pa(node_id):
    cw = MNEMONIC["MS"]
    # GET SP + PA: MS[1]
    err, resp = simplecan3_write_read(node_id, cw, 1, [1])
    if err == 0 and resp["dl"] >= 8:
        sp = int.from_bytes(resp["data"][2:5], byteorder="little", signed=True)  # d2,d3,d4
        pa = int.from_bytes(resp["data"][5:8], byteorder="little", signed=True)  # d5,d6,d7
        return {"speed": sp, "abs_pos": pa}
    return None

def stepper_clear_sf(node_id):
    cw = MNEMONIC["MS"]
    # CLEAR SF: MS[0]=0, data [0,0]
    err, resp = simplecan3_write_read(node_id, cw, 2, [0, 0])
    return err == 0

def stepper_get_motion_mode(node_id):
    cw = MNEMONIC["DV"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [0])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == 0:
        return resp["data"][2]  # 0=JOG, 1=PTP
    return None

def stepper_get_motor_current(node_id):
    cw = MNEMONIC["DV"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [1])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == 1:
        return resp["data"][2] * 0.1  # satuan 0.1A
    return None

def stepper_get_desired_sp(node_id):
    cw = MNEMONIC["DV"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [2])
    if err == 0 and len(resp["data"]) >= 6 and resp["data"][1] == 2:
        sp = int.from_bytes(resp["data"][2:6], byteorder="little", signed=True)
        return sp
    return None

def stepper_get_desired_pr(node_id):
    cw = MNEMONIC["DV"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [3])
    if err == 0 and len(resp["data"]) >= 6 and resp["data"][1] == 3:
        pr = int.from_bytes(resp["data"][2:6], byteorder="little", signed=True)
        return pr
    return None

def stepper_get_desired_pa(node_id):
    cw = MNEMONIC["DV"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [4])
    if err == 0 and len(resp["data"]) >= 6 and resp["data"][1] == 4:
        pa = int.from_bytes(resp["data"][2:6], byteorder="little", signed=True)
        return pa
    return None

def stepper_pvt_get_queue(node_id):
    """Get Current Queue Level (MP[0])"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [0])
    if err == 0 and len(resp["data"]) >= 4:
        return resp["data"][2] | (resp["data"][3] << 8)
    return None

def stepper_pvt_clear_queue(node_id):
    """Reset PVT Table (MP[0]=0)"""
    cw = MNEMONIC["MP"]
    # Set index 0 (queue), value 0 (clear)
    err, resp = simplecan3_write_read(node_id, cw, 3, [0, 0, 0])
    return err == 0

def stepper_pvt_set_first_valid_row(node_id, value):
    """Set First Valid Row in PVT Table (MP[1]=value)"""
    cw = MNEMONIC["MP"]
    # Index=1, value=uint16
    v_lo = value & 0xFF
    v_hi = (value >> 8) & 0xFF
    err, resp = simplecan3_write_read(node_id, cw, 3, [1, v_lo, v_hi])
    return err == 0

def stepper_pvt_get_first_valid_row(node_id):
    """Get First Valid Row in PVT Table (MP[1])"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [1])
    if err == 0 and len(resp["data"]) >= 4:
        return resp["data"][2] | (resp["data"][3] << 8)
    return None

def stepper_pvt_set_last_valid_row(node_id, value):
    """Set Last Valid Row in PVT Table (MP[2]=value)"""
    cw = MNEMONIC["MP"]
    v_lo = value & 0xFF
    v_hi = (value >> 8) & 0xFF
    err, resp = simplecan3_write_read(node_id, cw, 3, [2, v_lo, v_hi])
    return err == 0

def stepper_pvt_get_last_valid_row(node_id):
    """Get Last Valid Row in PVT Table (MP[2])"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [2])
    if err == 0 and len(resp["data"]) >= 4:
        return resp["data"][2] | (resp["data"][3] << 8)
    return None

def stepper_pvt_set_management_mode(node_id, mode):
    """Set PVT Data Management Mode (MP[3]=mode: 0=FIFO, 1=Single, 2=Loop)"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 3, [3, mode & 0xFF, 0])
    return err == 0

def stepper_pvt_get_management_mode(node_id):
    """Get PVT Data Management Mode (MP[3]): 0=FIFO, 1=Single, 2=Loop"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [3])
    if err == 0 and len(resp["data"]) >= 3:
        return resp["data"][2]  # HANYA 1 BYTE untuk mode
    return None

def stepper_pvt_set_pt_time(node_id, mode):
    """Set Time for PT Motion (MP[4]=mode: 0=PVT)"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 3, [4, mode & 0xFF, 0])
    return err == 0

def stepper_pvt_get_pt_time(node_id):
    """Get Time for PT Motion (MP[4])"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [4])
    if err == 0 and len(resp["data"]) >= 3:
        return resp["data"][2]
    return None

def stepper_pvt_set_queue_low(node_id, value):
    """Set Queue Low Alert Value (MP[5]=value)"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 3, [5, value & 0xFF, (value>>8) & 0xFF])
    return err == 0

def stepper_pvt_get_queue_low(node_id):
    """Get Queue Low Alert Value (MP[5])"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [5])
    if err == 0 and len(resp["data"]) >= 4:
        return resp["data"][2] | (resp["data"][3] << 8)
    return None

def stepper_pvt_set_next_available_writing_row(node_id, value):
    """Set Index of Next Available Writing Row (MP[6]=value)"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 3, [6, value & 0xFF, (value>>8) & 0xFF])
    return err == 0

def stepper_pvt_get_next_available_writing_row(node_id):
    """Get Index of Next Available Writing Row (MP[6])"""
    cw = MNEMONIC["MP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [6])
    if err == 0 and len(resp["data"]) >= 4:
        return resp["data"][2] | (resp["data"][3] << 8)
    return None

def stepper_pvt_set_position_row_n(node_id, row_n, position):
    """
    Set position value of row N in PVT table (QP[N]=position)
    - row_n: index PVT row (0..255)
    - position: signed 32 bit int (pulse)
    """
    cw = MNEMONIC["QP"]
    # Data: [row_n, pos_LSB, pos_2, pos_3, pos_MSB]
    pos_bytes = position.to_bytes(4, byteorder='little', signed=True)
    data = [row_n] + list(pos_bytes)
    err, resp = simplecan3_write_read(node_id, cw, 5, data)
    if err == 0 and resp["dl"] >= 5:
        return True
    return False

def stepper_pvt_get_position_row_n(node_id, n):
    cw = MNEMONIC["QP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [n])
    print(f"[DEBUG] Data={resp['data']}, len={len(resp['data'])}")
    if err == 0 and len(resp["data"]) >= 6 and resp["data"][1] == n:
        val = struct.unpack('<i', bytes(resp["data"][2:6]))[0]
        return val
    return None

def stepper_pvt_set_velocity_row_n(node_id, row, velocity):
    """
    Set velocity value at row N in PVT table.
    - row: index (0..255)
    - velocity: int32, pulses/sec (signed)
    Returns True if success.
    """
    cw = MNEMONIC["QV"]
    # Velocity to 4 bytes, little-endian signed
    v_bytes = velocity.to_bytes(4, byteorder='little', signed=True)
    data = [row] + list(v_bytes[:4])
    err, resp = simplecan3_write_read(node_id, cw, 5, data)
    return err == 0

def stepper_pvt_get_velocity_row_n(node_id, row):
    """
    Get velocity value at row N in PVT table.
    Returns signed int (velocity), or None if error.
    """
    cw = MNEMONIC["QV"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [row])
    # resp['data']: [CW, index, d1, d2, d3, d4, ...]
    if err == 0 and resp["dl"] >= 5 and resp["data"][0] == cw and resp["data"][1] == row:
        # d4:d3:d2:d1 little-endian (resp['data'][2:6])
        vel_bytes = bytes(resp["data"][2:6])
        velocity = int.from_bytes(vel_bytes, byteorder='little', signed=True)
        return velocity
    return None

def stepper_pvt_set_time_row_n(node_id, row, t_ms):
    # row = N, t_ms = value (0..65535), CW=0xA7
    # Data=[row, t_ms_LSB, t_ms_MSB]
    data = [row, t_ms & 0xFF, (t_ms >> 8) & 0xFF]
    err, resp = simplecan3_write_read(node_id, 0x27, 3, data)
    return err == 0

def stepper_pvt_get_time_row_n(node_id, row):
    # CW=0xA7, DL=1, Data=[row]
    err, resp = simplecan3_write_read(node_id, 0x27, 1, [row])
    if err == 0 and resp and len(resp["data"]) >= 4:
        # resp["data"] = [0x27, row, LSB, MSB, ...]
        value = resp["data"][2] + (resp["data"][3] << 8)
        return value
    return None

def stepper_pvt_set_quick_feeding(node_id, qp, qv, qt):
    """
    Kirim satu segment Quick Feeding (PVT) ke driver.
    - qp: Position (signed 32bit, pulses)
    - qv: Velocity (signed 24bit, pulses/sec)
    - qt: Time (unsigned 8bit, ms)
    """
    cw = 0x29  # QF
    data = [
        qt & 0xFF,                     # d0: QT (time)
        qv & 0xFF,                     # d1: QV LSB
        (qv >> 8) & 0xFF,              # d2: QV
        (qv >> 16) & 0xFF,             # d3: QV MSB (signed!)
        qp & 0xFF,                     # d4: QP LSB
        (qp >> 8) & 0xFF,              # d5: QP
        (qp >> 16) & 0xFF,             # d6: QP
        (qp >> 24) & 0xFF,             # d7: QP MSB
    ]
    err, resp = simplecan3_write_read(node_id, cw, 8, data)
    return err == 0

def stepper_pvt_get_quick_feeding_row_n(request_id, row_index):
    """
    Baca Quick Feeding PVT (QF[N]) untuk row ke-N
    Return: (QT, QV, QP) atau (None, None, None) jika gagal
    """
    cw = 0xA9
    dl = 1
    data = [row_index]

    # Call your CAN function
    resp = simplecan3_write_read(request_id, cw, dl, data)
    if not resp or len(resp) < 8:
        print(f"QF[{row_index}]: No/short response: {resp}")
        return None, None, None

    # resp[2]: QT, resp[3:6]: QV (signed 24bit), resp[6:10]: QP (signed 32bit)
    QT = resp[2]
    QV = int.from_bytes(resp[3:6], 'little', signed=True)
    # Cek apakah resp cukup panjang untuk QP
    if len(resp) >= 10:
        QP = int.from_bytes(resp[6:10], 'little', signed=True)
    else:
        # fallback (padding 0 jika cuma 8 byte)
        QP = int.from_bytes(resp[6:8] + [0, 0], 'little', signed=True)
    return QT, QV, QP