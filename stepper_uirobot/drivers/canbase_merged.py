"""
canbase_merged.py — UIM342AB SimpleCAN3 driver + high-level adapter (single file)

- Based on your canbase_patched.py (python-can / SocketCAN)
- Keeps low-level CW handling + ACK matching improvements
- Adds high-level helpers: RT/RT16, QE get/set, MS (Acquire Motion Status),
  and convenient read_position() that defaults to MS[0].
- Expands CLI: ping, get-id, set-id, mo/jv/sp/pr/pa/bg/stop, rt, rt16, read-pos,
  scan-rt, qe-get, qe-set, ms-read.

Usage examples:
    # Ping multiple nodes (Model Info)
    python canbase_merged.py --channel can0 ping 5 6 7

    # Read node id (PP[7])
    python canbase_merged.py --channel can0 get-id 6

    # Read position via MS (default)
    python canbase_merged.py --channel can0 read-pos 6

    # Read position via RT index (if your firmware supports RT for position)
    python canbase_merged.py --channel can0 read-pos 6 --via rt --idx 1

    # Motor on/off, jog velocity, stop
    python canbase_merged.py --channel can0 mo 6 on
    python canbase_merged.py --channel can0 jv 6 2000
    python canbase_merged.py --channel can0 stop 6

Notes:
- SocketCAN bitrate is configured externally, e.g.:
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up
- For RT: "idx" depends on vendor mapping. You can set env POS_IDX, default 1.
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
    "QE": 0x3D,
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
        """Wait for an ACK. Match CW via low 8-bit of CAN-ID OR data[0],
        and also recognize error frames that reference our CW.
        Accept both plain CW and CW|0x80 (ACK-bit set)."""
        want = base_cw & 0xFF
        want_ack = (want | 0x80) & 0xFF
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.bus.recv(timeout=timeout)
            if not msg:
                continue
            low_id = msg.arbitration_id & 0xFF
            data = bytes(msg.data)
            if self.debug:
                print(
                    f"DBG RX id=0x{msg.arbitration_id:08X} len={len(data)} "
                    f"data={data.hex(' ')} low_id=0x{low_id:02X}"
                )
            # 1) match via CAN-ID low byte
            if low_id in (want, want_ack):
                return msg
            # 2) match via payload byte-0 (echo CW)
            if len(data) >= 1 and data[0] in (want, want_ack):
                return msg
            # 3) ERROR frame referencing our CW (data[0]==ER, data[3]==CW)
            if len(data) >= 4 and data[0] == CW["ER"] and data[3] in (want, want_ack):
                return msg
        return None

    def transact(self, node_id: int, cw: int, data: bytes = b'', *, timeout: float = 0.6) -> Optional[can.Message]:
        """Send a command and wait for its ACK (when applicable)."""
        self.send(node_id, cw, data, ask_ack=True)
        if cw == CW["SY"]:
            return None  # No-ACK expected
        return self.recv_ack(cw, timeout=timeout)

    # ---- High-level helpers (existing & extended) ----
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
        # Pattern A: [idx, val]
        if len(d) >= 2 and d[0] == 0x07:
            return d[1]
        # Pattern B: [CW-echo, idx, val]
        if len(d) >= 3 and d[1] == 0x07:
            return d[2]
        # Pattern C: [val] only (fallback)
        if len(d) >= 1 and 5 <= d[0] <= 126:
            return d[0]
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
    # RT helpers
    # -----------------------------
    def rt(self, node_id: int, idx: int, timeout: float = 0.6) -> bytes:
        ack = self.transact(node_id, CW["RT"], pack_u8(idx), timeout=timeout)
        if not ack:
            raise TimeoutError("No ACK for RT")
        d = bytes(ack.data)
        # ERROR format: [0x0F, ec_lo, ec_hi, echo_cw, ...]
        if len(d) >= 4 and d[0] == CW["ER"]:
            err_le = int.from_bytes(d[1:3], 'little')
            cw_echo = d[3]
            raise RuntimeError(f"RT[{idx}] error 0x{err_le:04X} (ER for CW 0x{cw_echo:02X})")
        return d

    def rt16(self, node_id: int, idx: int, timeout: float = 0.6) -> bytes:
        payload = struct.pack('<H', idx & 0xFFFF)
        ack = self.transact(node_id, CW["RT"], payload, timeout=timeout)
        if not ack:
            raise TimeoutError("No ACK for RT16")
        d = bytes(ack.data)
        if len(d) >= 4 and d[0] == CW["ER"]:
            err_le = int.from_bytes(d[1:3], 'little')
            raise RuntimeError(f"RT16[{idx}] error 0x{err_le:04X}")
        return d

    # -----------------------------
    # QE helpers (encoder parameters)
    # -----------------------------
    def qe_get(self, node_id: int, idx: int, timeout: float = 0.8) -> int:
        ack = self.transact(node_id, CW["QE"], pack_u8(idx), timeout=timeout)
        if not ack:
            raise TimeoutError("No ACK for QE get")
        d = bytes(ack.data)
        # common patterns: [idx, lo, hi] or [CW-echo, idx, lo, hi]
        if len(d) >= 3 and d[0] == idx:
            return d[1] | (d[2] << 8)
        if len(d) >= 4 and d[1] == idx:
            return d[2] | (d[3] << 8)
        raise ValueError(f"Unexpected QE-get payload: {d.hex(' ')}")

    def qe_set(self, node_id: int, idx: int, value: int, timeout: float = 0.8) -> None:
        payload = pack_u8(idx) + pack_u16(value)
        ack = self.transact(node_id, CW["QE"], payload, timeout=timeout)
        if not ack:
            raise TimeoutError("No ACK for QE set")

    # -----------------------------
    # MS helpers (Acquire Motion Status)
    # -----------------------------
    @staticmethod
    def _slice_payload(data: bytes, cw_expected: int, needed_len: int) -> bytes:
        """Return the first `needed_len` bytes of the payload, handling optional CW echo at data[0]."""
        if len(data) >= needed_len + 1 and data[0] in (cw_expected, (cw_expected | 0x80) & 0xFF):
            return data[1 : 1 + needed_len]
        if len(data) >= needed_len:
            return data[0:needed_len]
        raise ValueError(f"Unexpected payload length {len(data)} for CW 0x{cw_expected:02X}: {data.hex(' ')}")

    def ms_read(self, node_id: int, idx: int = 0, timeout: float = 0.8) -> bytes:
        ack = self.transact(node_id, CW["MS"], pack_u8(idx), timeout=timeout)
        if not ack:
            raise TimeoutError("No ACK for MS")
        d = bytes(ack.data)
        if len(d) >= 4 and d[0] == CW["ER"]:
            err_le = int.from_bytes(d[1:3], 'little')
            raise RuntimeError(f"MS[{idx}] error 0x{err_le:04X}")
        # MS[0] expected to return 8 data bytes
        return self._slice_payload(d, CW["MS"], 8)

    def read_position_ms(self, node_id: int, idx: int = 0, timeout: float = 0.8) -> int:
        """Read relative position using MS[idx] (default MS[0]).
        According to manual, the 32-bit position is at bytes d4..d7 (little-endian)."""
        b8 = self.ms_read(node_id, idx=idx, timeout=timeout)
        pos = int.from_bytes(b8[4:8], 'little', signed=True)
        return pos

    # Unified convenience: default to MS; switch to RT via mode or env
    def read_position(self, node_id: int, idx: Optional[int] = None, timeout: float = 0.8, via: str = 'ms') -> int:
        if via.lower() == 'rt':
            if idx is None:
                idx = int(os.getenv("POS_IDX", "1"))
            d = self.rt(node_id, idx, timeout=timeout)
            # try decode int32 robustly
            if len(d) >= 5 and d[0] == CW["RT"]:
                raw = d[1:5]
            elif len(d) >= 4:
                raw = d[0:4]
            else:
                raise ValueError(f"Unexpected RT payload len={len(d)}: {d.hex(' ')}")
            return struct.unpack('<i', raw)[0]
        # default via MS[0]
        return self.read_position_ms(node_id, idx=0, timeout=timeout)

    # Alias for readability
    def read_encoder(self, node_id: int, idx: Optional[int] = None, timeout: float = 0.8, via: str = 'ms') -> int:
        return self.read_position(node_id, idx=idx, timeout=timeout, via=via)

    # Placeholder: vendor-specific zeroing (origin) may require different CW/PP index
    def set_origin_soft(self, node_id: int):
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

    # NEW: scan-rt (sweep index for RT)
    sp_scan = sub.add_parser('scan-rt', help='Scan RT indices and show responses')
    sp_scan.add_argument('id', type=int)
    sp_scan.add_argument('--start', type=int, default=0)
    sp_scan.add_argument('--end', type=int, default=31)

    # NEW: RT + RT16 + read-pos/read-encoder
    sp_rt = sub.add_parser('rt', help='Raw RT read with 1-byte index')
    sp_rt.add_argument('id', type=int)
    sp_rt.add_argument('--idx', type=int, required=True, help='RT index (0..255)')

    sp_rt16 = sub.add_parser('rt16', help='Raw RT read with 2-byte index')
    sp_rt16.add_argument('id', type=int)
    sp_rt16.add_argument('--idx', type=int, required=True)

    sp_rpos = sub.add_parser('read-pos', help='Read encoder position (default via MS[0])')
    sp_rpos.add_argument('id', type=int)
    sp_rpos.add_argument('--via', choices=['ms','rt'], default='ms', help='Select backend (ms|rt)')
    sp_rpos.add_argument('--idx', type=int, default=None, help='RT index for position (when --via rt)')

    sp_renc = sub.add_parser('read-encoder', help='Alias of read-pos')
    sp_renc.add_argument('id', type=int)
    sp_renc.add_argument('--via', choices=['ms','rt'], default='ms')
    sp_renc.add_argument('--idx', type=int, default=None)

    # MS raw read (debugging)
    sp_ms = sub.add_parser('ms-read', help='Raw MS read (returns 8 data bytes)')
    sp_ms.add_argument('id', type=int)
    sp_ms.add_argument('--idx', type=int, default=0)

    # QE get/set
    sp_qeg = sub.add_parser('qe-get', help='QE[i] get (encoder parameters)')
    sp_qeg.add_argument('id', type=int)
    sp_qeg.add_argument('idx', type=int)

    sp_qes = sub.add_parser('qe-set', help='QE[i] set (encoder parameters)')
    sp_qes.add_argument('id', type=int)
    sp_qes.add_argument('idx', type=int)
    sp_qes.add_argument('value', type=int)

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

    elif args.cmd == 'rt16':
        data = dev.rt16(args.id, args.idx)
        print(f"RT16[{args.idx}] @ ID {args.id} -> {data.hex(' ')}")

    elif args.cmd in ('read-pos', 'read-encoder'):
        pos = dev.read_position(args.id, idx=args.idx, via=args.via)
        print(f"POS @ ID {args.id} -> {pos}")

    elif args.cmd == 'scan-rt':
        def _parse_i32(d: bytes) -> Optional[int]:
            # robust parse: if payload starts with RT echo, skip it
            if len(d) >= 5 and d[0] == CW["RT"]:
                raw = d[1:5]
            elif len(d) >= 4:
                raw = d[0:4]
            else:
                return None
            return int.from_bytes(raw, 'little', signed=True)

        for i in range(args.start, args.end + 1):
            try:
                d = dev.rt(args.id, i)
                val = _parse_i32(d)
                if val is not None:
                    print(f"idx {i:02d}: int32={val}  raw={d.hex(' ')}")
                else:
                    print(f"idx {i:02d}: raw={d.hex(' ')} (len={len(d)})")
            except RuntimeError as e:
                print(f"idx {i:02d}: {e}")
            except TimeoutError:
                print(f"idx {i:02d}: timeout")

    elif args.cmd == 'ms-read':
        b8 = dev.ms_read(args.id, idx=args.idx)
        print(f"MS[{args.idx}] @ ID {args.id} -> {b8.hex(' ')}  (pos={int.from_bytes(b8[4:8],'little',signed=True)})")

    elif args.cmd == 'qe-get':
        val = dev.qe_get(args.id, args.idx)
        print(f"QE[{args.idx}] @ ID {args.id} = {val}")

    elif args.cmd == 'qe-set':
        # safety: recommend motor off before persistent write
        dev.mo(args.id, False)
        dev.qe_set(args.id, args.idx, args.value)
        print(f"QE[{args.idx}] @ ID {args.id} set to {args.value}")


if __name__ == '__main__':
    _cli()
