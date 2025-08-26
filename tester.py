from .uim342ab import *

br_raw = uim342ab_get_pp(6, 5)
BITRATE_MAP = {0: "1000Kbps", 1: "800Kbps", 2: "500Kbps", 3: "250Kbps", 4: "125Kbps"}
print("Bitrate =", BITRATE_MAP.get(br_raw, "Unknown"))

node = uim342ab_get_pp(6, 7)
group = uim342ab_get_pp(6, 8)
print("Node ID =", node)
print("Group ID =", group)