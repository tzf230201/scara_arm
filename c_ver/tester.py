from uim342ab import (
    uim342ab_get_bitrate,
    uim342ab_set_bitrate,
    uim342ab_get_node_id,
    uim342ab_set_node_id,
    uim342ab_get_group_id,
    uim342ab_set_group_id,
)

def print_yellow(text: str):
    print(f"\033[93m{text}\033[0m")

BITRATE_MAP = {
    0: "1000Kbps",
    1: "800Kbps",
    2: "500Kbps",
    3: "250Kbps",
    4: "125Kbps"
}

NODE_ID = 6  # ganti sesuai node yang kamu uji

# === BITRATE TEST ===
bitrate = uim342ab_get_bitrate(NODE_ID)
if bitrate is not None:
    print_yellow(f"[✓] Bitrate saat ini (PP[5]): {BITRATE_MAP.get(bitrate, 'Unknown')} ({bitrate})")
else:
    print_yellow("[✗] Gagal membaca bitrate")

# Contoh ubah ke 500Kbps (kode 2)
res = uim342ab_set_bitrate(NODE_ID, 2)
if res is not None:
    print_yellow(f"[✓] Bitrate berhasil diset ke {BITRATE_MAP.get(res, 'Unknown')} ({res})")
else:
    print_yellow("[✗] Gagal menyetel bitrate")


# === NODE ID TEST ===
node_id = uim342ab_get_node_id(NODE_ID)
if node_id is not None:
    print_yellow(f"[✓] Node ID saat ini (PP[7]): {node_id}")
else:
    print_yellow("[✗] Gagal membaca Node ID")

# Contoh set Node ID ke 6 lagi
res = uim342ab_set_node_id(NODE_ID, 6)
if res is not None:
    print_yellow(f"[✓] Node ID berhasil diset ke: {res}")
else:
    print_yellow("[✗] Gagal menyetel Node ID")


# === GROUP ID TEST ===
group_id = uim342ab_get_group_id(NODE_ID)
if group_id is not None:
    print_yellow(f"[✓] Group ID saat ini (PP[8]): {group_id}")
else:
    print_yellow("[✗] Gagal membaca Group ID")

# Contoh set Group ID ke 10
res = uim342ab_set_group_id(NODE_ID, 10)
if res is not None:
    print_yellow(f"[✓] Group ID berhasil diset ke: {res}")
else:
    print_yellow("[✗] Gagal menyetel Group ID")
