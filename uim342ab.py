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

def uim342ab_get_ptp_finish_notification(node_id):
    """
    IE[8] get → 0: disable, 1: enable
    ACK payload format (umumnya): [cmd, index, lo, hi, ...]
    """
    cw = MNEMONIC["IE"]
    index = 8
    err, resp = simplecan3_write_read(node_id, cw, 1, [index])
    if err == 0 and len(resp["data"]) >= 4 and resp["data"][1] == index:
        lo, hi = resp["data"][2], resp["data"][3]
        return (hi << 8) | lo   # return 0/1
    return None


def uim342ab_set_ptp_finish_notification(node_id, enable):
    """
    IE[8]=N set → N: 0 disable, 1 enable
    Catatan datasheet: nilai disimpan ke EEPROM hanya jika MO=0,
    kalau MO=1 nilai hanya di RAM dan hilang setelah power-off.
    """
    cw = MNEMONIC["IE"]
    index = 8
    val = 1 if enable else 0
    lo, hi = (val & 0xFF), ((val >> 8) & 0xFF)
    err, resp = simplecan3_write_read(node_id, cw, 3, [index, lo, hi])
    if err == 0 and len(resp["data"]) >= 4 and resp["data"][1] == index:
        rlo, rhi = resp["data"][2], resp["data"][3]
        return (rhi << 8) | rlo   # 0/1 yang terkonfirmasi
    return None

def uim342ab_get_error_report(node_id, index: int = 0):
    """
    ER[i] GET
      i = 0     → newest error info
      i = 10..18→ history 1st..9th (newest→oldest)
    Return dict: {"error_code", "cw_related", "sub_index", "raw"}
    """
    cw = MNEMONIC["ER"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [index])
    # Expect ACK data: [i, d1, d2, d3, d4, d5]
    if err == 0 and len(resp["data"]) >= 6 and resp["data"][0] == index:
        d = resp["data"]
        return {
            "error_code": d[1],   # per datasheet
            "cw_related": d[2],   # CW related to error
            "sub_index":  d[3],   # sub-index related
            "raw":        d[:6],  # keep first 6 for inspection
        }
    return None


def uim342ab_clear_error_report(node_id) -> bool:
    """
    ER[0] = 0  → Reset All Error Info
    """
    cw = MNEMONIC["ER"]
    index = 0
    value = 0
    err, resp = simplecan3_write_read(node_id, cw, 2, [index, value])
    # Expect ACK like: [0x00, 0, 0, 0, 0, 0]
    if err == 0 and len(resp["data"]) >= 2 and resp["data"][0] == 0:
        return True
    return False