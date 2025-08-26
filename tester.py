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

NODE_ID = 6  # ganti sesuai node ID steppermu

# === GET BITRATE ===
bitrate = uim342ab_get_bitrate(NODE_ID)
if bitrate is not None:
    print_yellow(f"[✓] Bitrate saat ini (PP[5]): {BITRATE_MAP.get(bitrate, 'Unknown')} ({bitrate})")
else:
    print_yellow("[✗] Gagal membaca bitrate")


# === GET NODE ID ===
node_id = uim342ab_get_node_id(NODE_ID)
if node_id is not None:
    print_yellow(f"[✓] Node ID saat ini (PP[7]): {node_id}")
else:
    print_yellow("[✗] Gagal membaca Node ID")


# === SET + GET GROUP ID ===
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


# === GET + SET AC/DC UNIT (IC[4]) ===
ac_unit = uim342ab_get_ac_dc_unit(NODE_ID)
unit_str = "pulse/sec²" if ac_unit == 0 else "milliseconds" if ac_unit == 1 else "Unknown"
print_yellow(f"[•] AC/DC unit (IC[4]) saat ini: {ac_unit} ({unit_str})")

updated = uim342ab_set_ac_dc_unit(NODE_ID, 1)
unit_str = "pulse/sec²" if updated == 0 else "milliseconds" if updated == 1 else "Unknown"
print_yellow(f"[✓] AC/DC unit diperbarui ke: {updated} ({unit_str})")


# === GET + SET USING CLOSED LOOP (IC[6]) ===
closed_loop = uim342ab_get_using_close_loop(NODE_ID)
loop_str = "Open-loop (tanpa encoder)" if closed_loop == 0 else "Closed-loop (dengan encoder)" if closed_loop == 1 else "Unknown"
print_yellow(f"[•] Closed-loop (IC[6]) saat ini: {closed_loop} ({loop_str})")

updated = uim342ab_set_using_close_loop(NODE_ID, 1)
loop_str = "Open-loop (tanpa encoder)" if updated == 0 else "Closed-loop (dengan encoder)" if updated == 1 else "Unknown"
print_yellow(f"[✓] Closed-loop diperbarui ke: {updated} ({loop_str})")
