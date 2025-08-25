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

def set_mo(node_id, enable: int):
    """
    Set motor driver ON/OFF. Return 1 jika ON, 0 jika OFF, None jika gagal.
    """
    cw = MNEMONIC["MO"]
    err, resp = simplecan3_write_read(node_id, cw, 1, [1 if enable else 0])
    if err == 0 and resp["dl"] > 0:
        return resp["data"][0]
    return None

def get_mo(node_id):
    """
    Get motor driver ON/OFF status. Return 1 jika ON, 0 jika OFF, None jika gagal.
    """
    cw = MNEMONIC["MO"]
    err, resp = simplecan3_write_read(node_id, cw, 0, [])
    if err == 0 and resp["dl"] > 0:
        return resp["data"][0]
    return None

set_mo(6, 1)  # Contoh: Set motor driver ON untuk node ID 6
status = get_mo(6)  # Contoh: Get status motor driver untuk node ID 6
print(f"Status motor driver untuk node ID 6: {status}")