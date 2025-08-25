# stepper_uirobot.py

from canbase_merged import simplecan3_write_read, NO_ERROR

def stepper_set_motor_on(node_id):
    """
    Aktifkan motor pada node_id (UIM342, SimpleCAN3).
    """
    # MO=1 (Motor ON), CW=0x95, DL=1, data=[1]
    err, resp = simplecan3_write_read(node_id, 0x95, 1, [1])
    if err == NO_ERROR:
        print(f"[ON] Motor pada ID {node_id} berhasil diaktifkan.")
    else:
        print(f"[ON] Gagal mengaktifkan motor ID {node_id}")

def stepper_set_motor_off(node_id):
    """
    Nonaktifkan motor pada node_id (UIM342, SimpleCAN3).
    """
    # MO=0 (Motor OFF), CW=0x95, DL=1, data=[0]
    err, resp = simplecan3_write_read(node_id, 0x95, 1, [0])
    if err == NO_ERROR:
        print(f"[OFF] Motor pada ID {node_id} berhasil dimatikan.")
    else:
        print(f"[OFF] Gagal mematikan motor ID {node_id}")
