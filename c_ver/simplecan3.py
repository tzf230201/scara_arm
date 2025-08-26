import can
import getpass

NO_ERROR = 0x00
MSG_ERROR = 0x01
TIMEOUT_ERROR = 0x02

RECV_WAIT = 0.5                

bus = None  # Global bus variable

def start_can():
    global bus  # Menandakan bahwa kita akan menggunakan variabel bus global
    user = getpass.getuser()
    if user == "peter":
        bus = can.Bus(channel='can0', interface='socketcan')
        print("CAN bus initialized")

def stop_can():
    global bus  # Menggunakan bus global
    if bus is not None:
        bus.shutdown()
        print("CAN bus shutdown")
    else:
        print("CAN bus not initialized")

start_can()


def simplecan3_write(id, cw, dl, data, ack=True):
    # Paksa ACK jika diminta
    cw_out = (cw | 0x80) if ack else (cw & 0x7F)

    # Hitung SID/EID sesuai SimpleCAN3.0
    sid = ((id << 1) & 0x003F) | 0x0100
    eid = (((id << 1) & 0x00C0) << 8) | (cw_out & 0xFF)
    can_id = (sid << 18) | eid

    # Validasi/padding data
    if not (0 <= dl <= 8):
        raise ValueError("dl harus 0..8")
    data_bytes = bytearray((data or [])[:dl])
    while len(data_bytes) < dl:
        data_bytes.append(0x00)

    msg = can.Message(arbitration_id=can_id,
                      data=data_bytes,
                      is_extended_id=True)

    try:
        bus.send(msg)
        print(f"[SENT] CAN-ID=0x{can_id:08X}, CW=0x{cw_out:02X}, DL={dl}, Data={list(data_bytes)}")
        return msg
    except can.CanError as e:
        print(f"[ERROR] Gagal kirim SimpleCAN3.0: {e}")
        return None

def simplecan3_read(request_id):
    error_code = NO_ERROR
    response_id = 0x04
    response = {
        "id": None,
        "cw": None,
        "dl": 0,
        "data": []
    }

    while (request_id != response_id):
        message = bus.recv(RECV_WAIT)
        if message is None:
            error_code = TIMEOUT_ERROR
            return error_code, response

        can_id = message.arbitration_id
        # Ambil ID dari bits 24-30 (7 bits sesuai layout SimpleCAN3)
        # pada sistem simplecan3,untuk melihat id mana yang menjawab, id responser terdapat pada producer id
        response_id = (can_id >> 24) & 0x7F

        # debug print
        print(f"[DEBUG] can_id=0x{can_id:X}, response_id={response_id}, data={list(message.data)}")

        if response_id != request_id:
            continue

        sid = (can_id >> 18) & 0x7FF
        eid = can_id & 0x3FFFF
        cw = eid & 0xFF
        dl = len(message.data)
        data = list(message.data[:dl])

        response["id"] = response_id
        response["cw"] = cw
        response["dl"] = dl
        response["data"] = data

    return error_code, response


def simplecan3_write_read(request_id, cw, dl, data):
    """
    Kirim data ke UIM342 dan tunggu balasan menggunakan protokol SimpleCAN3.0.

    Parameters:
        request_id : int       -> ID device (misalnya 6)
        cw         : int       -> Control Word
        dl         : int       -> Data length (1–8)
        data       : list[int] -> Data array (contoh: [0x01, 0x02])

    Returns:
        (error_code, response_dict)
    """
    # Kirim data
    simplecan3_write(request_id, cw, dl, data, ack=True)

    # Tunggu dan baca balasan
    error_code, response = simplecan3_read(request_id)

    return error_code, response
