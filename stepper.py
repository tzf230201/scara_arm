# stepper.py

from canbase import simplecan3_write_read

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

