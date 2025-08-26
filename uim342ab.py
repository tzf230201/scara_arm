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

def uim342ab_get_bitrate(node_id):
    cw = MNEMONIC["PP"]
    index = 5
    err, resp = simplecan3_write_read(node_id, cw, 1, [index])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == index:
        return resp["data"][2]
    return None


def uim342ab_set_bitrate(node_id, value):
    cw = MNEMONIC["PP"]
    index = 5
    err, resp = simplecan3_write_read(node_id, cw, 3, [index, value, 0])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == index:
        return resp["data"][2]
    return None


def uim342ab_get_node_id(node_id):
    cw = MNEMONIC["PP"]
    index = 7
    err, resp = simplecan3_write_read(node_id, cw, 1, [index])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == index:
        return resp["data"][2]
    return None


def uim342ab_set_node_id(node_id, value):
    cw = MNEMONIC["PP"]
    index = 7
    err, resp = simplecan3_write_read(node_id, cw, 3, [index, value, 0])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == index:
        return resp["data"][2]
    return None


def uim342ab_get_group_id(node_id):
    cw = MNEMONIC["PP"]
    index = 8
    err, resp = simplecan3_write_read(node_id, cw, 1, [index])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == index:
        return resp["data"][2]
    return None


def uim342ab_set_group_id(node_id, value):
    cw = MNEMONIC["PP"]
    index = 8
    err, resp = simplecan3_write_read(node_id, cw, 2, [index, value])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == index:
        return resp["data"][2]
    return None


def uim342ab_get_ac_dc_unit(node_id):
    cw = MNEMONIC["IC"]
    index = 4
    err, resp = simplecan3_write_read(node_id, cw, 1, [index])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == index:
        return resp["data"][2]
    return None


def uim342ab_set_ac_dc_unit(node_id, value):
    cw = MNEMONIC["IC"]
    index = 4
    err, resp = simplecan3_write_read(node_id, cw, 3, [index, value, 0])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == index:
        return resp["data"][2]
    return None


def uim342ab_get_using_close_loop(node_id):
    cw = MNEMONIC["IC"]
    index = 6
    err, resp = simplecan3_write_read(node_id, cw, 1, [index])
    if err == 0 and len(resp["data"]) >= 3 and resp["data"][1] == index:
        return resp["data"][2]  
    return None