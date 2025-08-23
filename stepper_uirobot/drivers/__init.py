# drivers/__init__.py
"""
drivers package
- Ekspor kelas UIM342CAN dari canbase_merged
- Sediakan helper get_driver() biar gampang init via env
"""

import os
from .canbase_merged import UIM342CAN, CW, NODE_ID_MIN, NODE_ID_MAX

__all__ = [
    "UIM342CAN",
    "CW",
    "NODE_ID_MIN",
    "NODE_ID_MAX",
    "get_driver",
]

def get_driver(*, debug: bool = False, channel: str | None = None) -> UIM342CAN:
    """
    Buat instance driver dengan channel dari argumen atau ENV:
      - CAN_CHANNEL (default: 'can0')
      - POS_IDX bisa di-set di ENV tapi dipakai di dalam UIM342CAN.read_position()
    """
    ch = channel or os.getenv("CAN_CHANNEL", "can0")
    return UIM342CAN(channel=ch, debug=debug)
