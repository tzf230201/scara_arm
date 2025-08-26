from uim342ab import *

BITRATE_MAP = {
    0: "1000Kbps",
    1: "800Kbps",
    2: "500Kbps",
    3: "250Kbps",
    4: "125Kbps"
}

NODE_ID = 6  # ganti dengan node ID targetmu

# === BITRATE TEST ===
# # Contoh set bitrate ke 500Kbps (kode 2)
# result = uim342ab_set_bitrate(NODE_ID, 2)
# if result is not None:
#     print(f"[✓] Bitrate berhasil diset ke {BITRATE_MAP.get(result, 'Unknown')} ({result})")
# else:
#     print("[✗] Gagal menyetel bitrate")

bitrate = uim342ab_get_bitrate(NODE_ID)
if bitrate is not None:
    print(f"[✓] Bitrate saat ini (PP[5]): {BITRATE_MAP.get(bitrate, 'Unknown')} ({bitrate})")
else:
    print("[✗] Gagal membaca bitrate")


# === NODE ID TEST ===
node_id = uim342ab_get_node_id(NODE_ID)
if node_id is not None:
    print(f"[✓] Node ID saat ini (PP[7]): {node_id}")
else:
    print("[✗] Gagal membaca Node ID")

# === GROUP ID TEST ===

# Contoh set group ID ke 10
result = uim342ab_set_group_id(NODE_ID, 10)
if result is not None:
    print(f"[✓] Group ID berhasil diset ke: {result}")
else:
    print("[✗] Gagal menyetel Group ID")

group_id = uim342ab_get_group_id(NODE_ID)
if group_id is not None:
    print(f"[✓] Group ID saat ini (PP[8]): {group_id}")
else:
    print("[✗] Gagal membaca Group ID")


node_id = 6

# Test get/set AC-DC Unit
current = uim342ab_get_ac_dc_unit(node_id)
print(f"Current AC/DC unit (IC[4]): {current}")

updated = uim342ab_set_ac_dc_unit(node_id, 1)  # 0: pulse/sec², 1: milliseconds
print(f"Updated AC/DC unit (IC[4]): {updated}")

# Test get/set Using Closed-loop
current = uim342ab_get_using_close_loop(node_id)
print(f"Current Closed-loop (IC[6]): {current}")

updated = uim342ab_set_using_close_loop(node_id, 1)  # 0: open loop, 1: closed loop
print(f"Updated Closed-loop (IC[6]): {updated}")
