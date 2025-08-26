from uim342ab import *

def print_yellow(text):
    YELLOW = "\033[93m"
    RESET = "\033[0m"
    print(f"{YELLOW}{text}{RESET}")

BITRATE_MAP = {
    0: "1000Kbps",
    1: "800Kbps",
    2: "500Kbps",
    3: "250Kbps",
    4: "125Kbps"
}

NODE_ID = 6  # Ganti sesuai target

# === BITRATE TEST ===
bitrate = uim342ab_get_bitrate(NODE_ID)
if bitrate is not None:
    print_yellow(f"[✓] Bitrate saat ini (PP[5]): {BITRATE_MAP.get(bitrate, 'Unknown')} ({bitrate})")
else:
    print_yellow("[✗] Gagal membaca bitrate")

# Contoh set bitrate ke 500Kbps (kode 2)
# result = uim342ab_set_bitrate(NODE_ID, 2)
# if result is not None:
#     print_yellow(f"[✓] Bitrate berhasil diset ke {BITRATE_MAP.get(result, 'Unknown')} ({result})")
# else:
#     print_yellow("[✗] Gagal menyetel bitrate")


# === NODE ID TEST ===
node_id = uim342ab_get_node_id(NODE_ID)
if node_id is not None:
    print_yellow(f"[✓] Node ID saat ini (PP[7]): {node_id}")
else:
    print_yellow("[✗] Gagal membaca Node ID")


# === GROUP ID TEST ===
result = uim342ab_set_group_id(NODE_ID, 10)
if result is not None:
    print_yellow(f"[✓] Group ID berhasil diset ke: {result}")
else:
    print_yellow("[✗] Gagal menyetel Group ID")

group_id = uim342ab_get_group_id(NODE_ID)
if group_id is not None:
    print_yellow(f"[✓] Group ID saat ini (PP[8]): {group_id}")
else:
    print_yellow("[✗] Gagal membaca Group ID")


# === AC/DC UNIT TEST (IC[4]) ===
acdc = uim342ab_get_ac_dc_unit(NODE_ID)
print_yellow(f"[•] AC/DC unit (IC[4]) saat ini: {acdc}")

acdc_set = uim342ab_set_ac_dc_unit(NODE_ID, 0)  # 0: pulse/sec², 1: milliseconds
print_yellow(f"[✓] AC/DC unit diperbarui ke: {acdc_set}")


# === CLOSED-LOOP TEST (IC[6]) ===
closed_loop = uim342ab_get_using_close_loop(NODE_ID)
print_yellow(f"[•] Closed-loop (IC[6]) saat ini: {closed_loop}")

closed_loop_set = uim342ab_set_using_close_loop(NODE_ID, 1)  # 0: open loop, 1: closed loop
print_yellow(f"[✓] Closed-loop diperbarui ke: {closed_loop_set}")
