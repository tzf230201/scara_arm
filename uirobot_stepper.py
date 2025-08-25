from canbase_merged import *

def simplecan3_ping(id, timeout=0.5):
    """
    Kirim command MO=1 dengan bit ACK aktif (CW=0x95 | 0x80), lalu tunggu balasan dari UIM342.
    
    Parameters:
        id      : int → node ID target (misal 5)
        timeout : float → waktu tunggu ACK (detik)
    """
    # CW untuk MO = 0x95 → tambahkan bit7 (ACK) jadi 0x95 | 0x80 = 0xD5
    cw = 0x95 | 0x80
    dl = 1
    data = [0x01]  # Aktifkan motor

    # Kirim perintah MO dengan ACK
    simplecan3_write_sdo(id, cw, dl, data)

    # Tunggu ACK dengan recv
    response = bus.recv(timeout=timeout)
    if response:
        print(f"[ACK RECEIVED] CAN-ID: 0x{response.arbitration_id:08X}, DATA: {list(response.data)}")
        return True
    else:
        print("[TIMEOUT] Tidak ada ACK dari node", id)
        return False
    

success = simplecan3_ping(6)
if success:
    print("✅ UIM342 dengan ID 5 aktif dan merespons.")
else:
    print("❌ Tidak ada respon dari UIM342 ID 5.")