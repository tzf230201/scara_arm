from simplecan3 import simplecan3_write_read
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


# === Bitrate ===
def uim342ab_get_bitrate(node_id):
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [5])
    if err == 0 and resp["dl"] >= 2 and resp["data"][0] == 5:
        return resp["data"][1]  # 0:1000K, 1:800K, 2:500K, 3:250K, 4:125K
    return None

def uim342ab_set_bitrate(node_id, bitrate_code):
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 2, [5, bitrate_code])
    if err == 0 and resp["dl"] >= 2 and resp["data"][0] == 5:
        return resp["data"][1]
    return None

# === Node ID ===
def uim342ab_get_node_id(node_id):
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [7])
    if err == 0 and resp["dl"] >= 2 and resp["data"][0] == 7:
        return resp["data"][1]
    return None

def uim342ab_set_node_id(node_id, new_node_id):
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 2, [7, new_node_id])
    if err == 0 and resp["dl"] >= 2 and resp["data"][0] == 7:
        return resp["data"][1]
    return None

# === Group ID ===
def uim342ab_get_group_id(node_id):
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [8])
    if err == 0 and resp["dl"] >= 2 and resp["data"][0] == 8:
        return resp["data"][1]
    return None

def uim342ab_set_group_id(node_id, new_group_id):
    cw = MNEMONIC["PP"]
    err, resp = simplecan3_write_read(node_id, cw, 2, [8, new_group_id])
    if err == 0 and resp["dl"] >= 2 and resp["data"][0] == 8:
        return resp["data"][1]
    return None
