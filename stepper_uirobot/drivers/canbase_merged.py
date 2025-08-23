"""
canbase_merged.py — UIM342AB SimpleCAN3 driver + high-level adapter (single file)

- Based on your canbase_patched.py (python-can / SocketCAN)
- Keeps low-level CW handling + ACK matching improvements
- Adds high-level helpers like RT() and read_position()
- Expands CLI with rt/read-pos/read-encoder, while preserving existing commands

Usage examples:
    # Ping multiple nodes (Model Info)
    python canbase_merged.py --channel can0 ping 5 6 7

    # Read node id (PP[7])
    python canbase_merged.py --channel can0 get-id 6

    # Read position via RT (idx=1 by default; adjust if your firmware uses another index)
    python canbase_merged.py --channel can0 read-pos 6 --idx 1

    # Motor on/off, jog velocity, stop
    python canbase_merged.py --channel can0 mo 6 on
    python canbase_merged.py --channel can0 jv 6 2000
    python canbase_merged.py --channel can0 stop 6

Notes:
- SocketCAN bitrate is configured externally, e.g.:
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up
- "idx" for RT depends on vendor mapping. Default is 1. Change with --idx or POS_IDX env.
- SY (0x7E) is No-ACK per manual.
"""
from __future__ import annotations
import argparse
import os
import struct
import time
from typing import Optional, Tuple

import can

# -----------------------------
# Constants (Control Words)
# -----------------------------
CW = {
    # Protocol & system
    "PP": 0x01,
    "IC": 0x06,
    "IE": 0x07,
    "ML": 0x0B,
    "SN": 0x0C,
    "ER": 0x0F,
    "MS": 0x11,
    "MO": 0x15,
    "BG": 0x16,
    "AC": 0x19,
    "DC": 0x1A,
    "SS": 0x1B,
    "SD": 0x1C,
    "JV": 0x1D,
    "SP": 0x1E,
    "PR": 0x1F,
    "PA": 0x20,
    "MP": 0x22,
    "PT": 0x23,
    "PV": 0x24,
    "QP": 0x25,
    "QV": 0x26,
    "QT": 0x27,
    "QF": 0x29,
    "DV": 0x2E,
    "IL": 0x34,
    "TG": 0x35,
    "DI": 0x37,
    "RT": 0x5A,
    "SY": 0x7E,  # No-ACK per manual
}

NODE_ID_MIN = 5
NODE_ID_MAX = 126

# -----------------------------
# Helpers
# -----------------------------

def need_ack(cw: int) -> int:
    """Return the host-side CW (set high bit to ask for ACK) when applicable."""
    if cw == CW["SY"]:
        # SY is No-ACK per manual
        return cw
    return (cw | 0x80) & 0xFF


def build_can_id(node_id: int, cw: int) -> int:
    """Compute 29-bit CAN ID for SimpleCAN3.0 (node/consumer ID = node_id)."""
    if not (0 <= node_id <= 127):
        raise ValueError("node_id must be 0..127 (device accepts 5..126)")
    sid = ((node_id << 1) & 0x3F) | 0x0100
    eid = (((node_id << 1) & 0x0C0) << 8) | (cw & 0xFF)
    return ((sid << 18) | eid) & 0x1FFFFFFF


def pack_s32(v: int) -> bytes:
    return struct.pack('<i', int(v))


def pack_u32(v: int) -> bytes:
    return struct.pack('<I', int(v) & 0xFFFFFFFF)


def pack_u16(v: int) -> bytes:
    return struct.pack('<H', int(v) & 0xFFFF)


def pack_u8(v: int) -> bytes:
    return struct.pack('<B', int(v) & 0xFF)


# -----------------------------
# Core class (+ high-level adapter merged)
# -----------------------------
class UIM342CAN:
    def __init__(self, channel: str = 'can0', bitrate: Optional[int] = None, debug: bool = False):
        """Open a SocketCAN interface. Set `bitrate` externally via ip link.
        Example: sudo ip link set can0 up type can bitrate 500000
        """
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan')
        self.debug = debug
        self.channel = channel

    # ---- Low-level send/recv ----
    def send(self, node_id: int, cw: int, data: bytes = b'', *, ask_ack: bool = True) -> None:
        tx_cw = need_ack(cw) if ask_ack else cw
        can_id = build_can_id(node_id, tx_cw)
        msg = can.Message(arbitration_id=can_id, is_extended_id=True, data=data)
        if self.debug:
            print(f"DBG TX id=0x{can_id:08X} cw=0x{tx_cw:02X} len={len(data)} data={data.hex(' ')}")
        self.bus.send(msg)

def recv_ack(self, base_cw: int, timeout: float = 0.6) -> Optional[can.Message]:
    want = base_cw & 0xFF
    want_ack = (want | 0x80) & 0xFF  # terima juga CW|0x80 (mis. 0xDA utk RT)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = self.bus.recv(timeout=timeout)
        if not msg:
            continue
        low_id = msg.arbitration_id & 0xFF
        data = bytes(msg.data)
        if self.debug:
            print(f"DBG RX id=0x{msg.arbitration_id:08X} len={len(data)} data={data.hex(' ')} low_id=0x{low_id:02X}")
        # 1) match lewat CAN-ID low byte
        if low_id in (want, want_ack):
            return msg
        # 2) match lewat payload byte-0 (echo CW)
        if len(data) >= 1 and data[0] in (want, want_ack):
            return msg
        # 3) match pola ERROR: data[0]==ER dan data[3] adalah CW yang kita minta
        if len(data) >= 4 and data[0] == CW["ER"] and data[3] in (want, want_ack):
            return msg
    return None



    def transact(self, node_id: int, cw: int, data: bytes = b'', *, timeout: float = 0.6) -> Optional[can.Message]:
        """Send a command and wait for its ACK (when applicable)."""
        self.send(node_id, cw, data, ask_ack=True)
        if cw == CW["SY"]:
            return None  # No-ACK expected
        return self.recv_ack(cw, timeout=timeout)

    # ---- High-level helpers (existing) ----
    def ping_ml(self, node_id: int) -> Optional[Tuple[int, bytes]]:
        """Ping using ML (Model Info). Returns (cw, data) from ACK or None.
        Accept CW from CAN-ID low byte or from data[0]."""
        ack = self.transact(node_id, CW["ML"], b"", timeout=0.8)
        if not ack:
            return None
        low_id = ack.arbitration_id & 0xFF
        data = bytes(ack.data)
        cw = data[0] if (len(data) >= 1) else low_id
        return (cw, data)

    def get_sn(self, node_id: int) -> Optional[bytes]:
        ack = self.transact(node_id, CW["SN"], b"", timeout=0.8)
        return bytes(ack.data) if ack else None

    # PP[7] Node ID
    def get_node_id(self, node_id: int) -> Optional[int]:
        ack = self.transact(node_id, CW["PP"], pack_u8(0x07), timeout=0.8)
        if not ack:
            return None
        d = bytes(ack.data)
        # Pola 1: [idx, val]
        if len(d) >= 2 and d[0] == 0x07:
            return d[1]
        # Pola 2: [CW-echo, idx, val]
        if len(d) >= 3 and d[1] == 0x07:
            return d[2]
        # Pola 3: [val] saja (fallback), valid kalau 5..126
        if len(d) >= 1 and 5 <= d[0] <= 126:
            return d[0]
        # Tidak dikenali
        if self.debug:
            print(f"get_node_id: unrecognized payload {d.hex(' ')}")
        return None


    def set_node_id(self, current_id: int, new_id: int) -> None:
        if not (NODE_ID_MIN <= new_id <= NODE_ID_MAX):
            raise ValueError(f"new_id must be {NODE_ID_MIN}..{NODE_ID_MAX}")
        # Ensure motor off so params can be saved to EEPROM
        self.mo(current_id, False)
        # PP[7] = new_id
        self.transact(current_id, CW["PP"], pack_u8(0x07) + pack_u8(new_id), timeout=0.8)
        # Reboot to take effect
        self.sy_reboot(current_id)

    # SY[1] reboot (No-ACK)
    def sy_reboot(self, node_id: int) -> None:
        self.send(node_id, CW["SY"], pack_u8(0x01), ask_ack=False)

    # MO: motor on/off
    def mo(self, node_id: int, enable: bool) -> None:
        self.transact(node_id, CW["MO"], pack_u8(0x01 if enable else 0x00))

    # BG: begin/apply motion parameters
    def bg(self, node_id: int) -> None:
        self.transact(node_id, CW["BG"], b"")

    # ST/SD: stop (uses SD decel)
    def stop(self, node_id: int) -> None:
        self.transact(node_id, CW["SD"], b"")

    # JV/SP: speed
    def jv(self, node_id: int, pps: int) -> None:
        self.transact(node_id, CW["JV"], pack_s32(pps))

    def sp(self, node_id: int, pps: int) -> None:
        self.transact(node_id, CW["SP"], pack_s32(pps))

    # PR/PA: position motions (pulse)
    def pr(self, node_id: int, pulses: int) -> None:
        self.transact(node_id, CW["PR"], pack_s32(pulses))

    def pa(self, node_id: int, pulses: int) -> None:
        self.transact(node_id, CW["PA"], pack_s32(pulses))

    # AC/DC/SS/SD
    def ac(self, node_id: int, val: int) -> None:
        self.transact(node_id, CW["AC"], pack_s32(val))

    def dc(self, node_id: int, val: int) -> None:
        self.transact(node_id, CW["DC"], pack_s32(val))

    def ss(self, node_id: int, val: int) -> None:
        self.transact(node_id, CW["SS"], pack_s32(val))

    def sd(self, node_id: int, val: int) -> None:
        self.transact(node_id, CW["SD"], pack_s32(val))

    # ------------- PVT/PT minimal helpers -------------
    def mp(self, node_id: int, idx: int, val: int) -> None:
        # MP[i] set (i, val) → 2 bytes
        self.transact(node_id, CW["MP"], pack_u8(idx) + pack_u8(val & 0xFF))

    def pv_set_start(self, node_id: int, start_row: int) -> None:
        self.transact(node_id, CW["PV"], pack_u16(start_row))

    def qp(self, node_id: int, row: int, pos: int) -> None:
        self.transact(node_id, CW["QP"], pack_u16(row) + pack_s32(pos))

    def qv(self, node_id: int, row: int, vel_pps: int) -> None:
        self.transact(node_id, CW["QV"], pack_u16(row) + pack_s32(vel_pps))

    def qt(self, node_id: int, row: int, t_ms: int) -> None:
        self.transact(node_id, CW["QT"], pack_u16(row) + pack_u16(t_ms))

    def qf(self, node_id: int, row: int, t_ms: int, vel_pps: int, pos: int) -> None:
        payload = pack_u16(row) + pack_u8(t_ms) + pack_s32(vel_pps) + pack_s32(pos)
        self.transact(node_id, CW["QF"], payload)

    # -----------------------------
    # Added high-level adapter helpers (merged in)
    # -----------------------------
    def rt(self, node_id: int, idx: int, timeout: float = 0.6) -> bytes:
        ack = self.transact(node_id, CW["RT"], pack_u8(idx), timeout=timeout)
        if not ack:
            raise TimeoutError("No ACK for RT")
        d = bytes(ack.data)
        # jika ERROR, format yang terlihat: [0x0F, ec_low, ec_high, echo_cw, ...]
        if len(d) >= 4 and d[0] == CW["ER"]:
            err = d[1] | (d[2] << 8)
            cw_echo = d[3]
            raise RuntimeError(f"RT[{idx}] error 0x{err:04X} (ER for CW 0x{cw_echo:02X})")
        return d


    def read_position(self, node_id: int, idx: Optional[int] = None, timeout: float = 0.6) -> int:
        """Read current encoder position (int32) via RT.
        - Default idx from POS_IDX env (default 1). Adjust to your firmware mapping.
        - Robust payload parse: if data[0]==CW['RT'], take data[1:5]; else data[0:4]."""
        if idx is None:
            idx = int(os.getenv("POS_IDX", "1"))
        data = self.rt(node_id, idx, timeout=timeout)
        if len(data) >= 5 and data[0] == CW["RT"]:
            raw = data[1:5]
        elif len(data) >= 4:
            raw = data[0:4]
        else:
            raise ValueError(f"Unexpected RT payload len={len(data)}: {data.hex(' ')}")
        return struct.unpack('<i', raw)[0]

    # Alias for readability
    def read_encoder(self, node_id: int, idx: Optional[int] = None, timeout: float = 0.6) -> int:
        return self.read_position(node_id, idx=idx, timeout=timeout)

    # Placeholder: vendor-specific zeroing (origin) may require different CW/PP index
    def set_origin_soft(self, node_id: int):
        """Not implemented: depends on vendor mapping (e.g., dedicated CW or PP index).
        This is intentionally a stub to avoid unsafe writes without the exact spec."""
        raise NotImplementedError("set_origin_soft requires vendor-specific object; not implemented")


# -----------------------------
# CLI
# -----------------------------

def _cli():
    p = argparse.ArgumentParser(description="UIM342AB SimpleCAN3.0 helper (merged)")
    p.add_argument('--channel', default='can0', help='SocketCAN channel (default: can0)')
    p.add_argument('--debug', action='store_true', help='Print TX/RX frames and relaxed ACK matching')
    sub = p.add_subparsers(dest='cmd', required=True)

    sp_ping = sub.add_parser('ping', help='Ping with ML (Model Info)')
    sp_ping.add_argument('ids', nargs='+', type=int)

    sp_getid = sub.add_parser('get-id', help='Read Node ID (PP[7])')
    sp_getid.add_argument('id', type=int)

    sp_setid = sub.add_parser('set-id', help='Set Node ID via PP[7], then reboot (SY[1])')
    sp_setid.add_argument('current', type=int)
    sp_setid.add_argument('new', type=int)

    sp_mo = sub.add_parser('mo', help='Motor on/off')
    sp_mo.add_argument('id', type=int)
    sp_mo.add_argument('state', choices=['on', 'off'])

    sp_jv = sub.add_parser('jv', help='Jog velocity (pps) + BG to apply (usually)')
    sp_jv.add_argument('id', type=int)
    sp_jv.add_argument('pps', type=int)

    sp_sp = sub.add_parser('sp', help='PTP speed (pps)')
    sp_sp.add_argument('id', type=int)
    sp_sp.add_argument('pps', type=int)

    sp_pr = sub.add_parser('pr', help='Position relative (pulses)')
    sp_pr.add_argument('id', type=int)
    sp_pr.add_argument('pulses', type=int)

    sp_pa = sub.add_parser('pa', help='Position absolute (pulses)')
    sp_pa.add_argument('id', type=int)
    sp_pa.add_argument('pulses', type=int)

    sp_bg = sub.add_parser('bg', help='Begin/apply motion')
    sp_bg.add_argument('id', type=int)

    sp_stop = sub.add_parser('stop', help='Stop (SD deceleration)')
    sp_stop.add_argument('id', type=int)

    # NEW: RT + read-pos/read-encoder
    sp_rt = sub.add_parser('rt', help='Raw RT read with 1-byte index')
    sp_rt.add_argument('id', type=int)
    sp_rt.add_argument('--idx', type=int, required=True, help='RT index (0..255)')

    sp_rpos = sub.add_parser('read-pos', help='Read encoder position via RT (int32)')
    sp_rpos.add_argument('id', type=int)
    sp_rpos.add_argument('--idx', type=int, default=None, help='RT index for position (default from POS_IDX env or 1)')

    sp_renc = sub.add_parser('read-encoder', help='Alias of read-pos')
    sp_renc.add_argument('id', type=int)
    sp_renc.add_argument('--idx', type=int, default=None)

    args = p.parse_args()

    dev = UIM342CAN(channel=args.channel, debug=args.debug)

    if args.cmd == 'ping':
        for nid in args.ids:
            resp = dev.ping_ml(nid)
            if resp:
                cw, data = resp
                print(f"ID {nid}: ACK CW=0x{cw:02X} DATA={data.hex(' ')}")
            else:
                print(f"ID {nid}: no response")

    elif args.cmd == 'get-id':
        val = dev.get_node_id(args.id)
        print(f"PP[7] @ ID {args.id} = {val}")

    elif args.cmd == 'set-id':
        dev.set_node_id(args.current, args.new)
        print(f"Node ID changed {args.current} -> {args.new} (after reboot)")

    elif args.cmd == 'mo':
        dev.mo(args.id, args.state == 'on')
        print(f"MO @ ID {args.id} = {args.state}")

    elif args.cmd == 'jv':
        dev.jv(args.id, args.pps)
        print(f"JV @ ID {args.id} = {args.pps} pps")

    elif args.cmd == 'sp':
        dev.sp(args.id, args.pps)
        print(f"SP @ ID {args.id} = {args.pps} pps")

    elif args.cmd == 'pr':
        dev.pr(args.id, args.pulses)
        print(f"PR @ ID {args.id} = {args.pulses} pulses")

    elif args.cmd == 'pa':
        dev.pa(args.id, args.pulses)
        print(f"PA @ ID {args.id} = {args.pulses} pulses")

    elif args.cmd == 'bg':
        dev.bg(args.id)
        print(f"BG @ ID {args.id}")

    elif args.cmd == 'stop':
        dev.stop(args.id)
        print(f"STOP (SD) @ ID {args.id}")

    elif args.cmd == 'rt':
        data = dev.rt(args.id, args.idx)
        print(f"RT[{args.idx}] @ ID {args.id} -> {data.hex(' ')}")

    elif args.cmd in ('read-pos', 'read-encoder'):
        pos = dev.read_position(args.id, idx=args.idx)
        print(f"POS @ ID {args.id} -> {pos}")


if __name__ == '__main__':
    _cli()
