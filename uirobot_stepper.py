# stepper_uirobot.py

from canbase_merged import simplecan3_write_read

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

BITRATE_MAP = {
    0: "1000Kbps",
    1: "800Kbps",
    2: "500Kbps",
    3: "250Kbps",
    4: "125Kbps"
}

def stepper_set_node_id(node_id, new_node_id):
    """
    Set CAN Node ID stepper.
    :param node_id: node lama (ID device sekarang)
    :param new_node_id: node baru yang di-set (1-127)
    :return: node_id baru jika sukses, None jika gagal
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