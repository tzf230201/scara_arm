# canbase — UIM342AB SimpleCAN 3.0 Helper (patched)

A small helper/utility to talk to **UIROBOT UIM342AB** stepper drives over **SimpleCAN 3.0** on **CAN 2.0B**. It implements the 29‑bit CAN‑ID encoding, a minimal command set (ping, read/set Node ID, MO/JV/SP/PR/PA/BG/STOP), and both **CLI** and **Python API** examples.

> **Important notes**
> - UIM342AB **does not use CANopen**. The protocol is **SimpleCAN 3.0** (proprietary), little‑endian payload.
> - Valid **Node ID range is 5…126** (0 = global/broadcast; 1–4 reserved/producer).
> - This README targets the **patched helper**: `canbase_patched.py` (robust ACK + `--debug`). You can rename it to `canbase.py` if you want.

---

## Changelog
**2025‑08‑22**
- Added robust ACK matching (accept CW from **CAN‑ID low byte** *or* from **data[0]**).
- Added `--debug` flag (prints TX/RX frames and helps diagnose mismatches).

---

## 1) Prerequisites
- Linux with **SocketCAN** (and `can-utils`: `candump`, `cansend`, `cansniffer`).
- Python 3.8+ and **python-can**.
- A working CAN network (common ground, **120 Ω** terminators at both ends, correct bitrate).

### Install python-can
```bash
pip install python-can
```

### Bring up SocketCAN
Adjust `500000` to your bus (125000/250000/500000/1000000):
```bash
sudo ip link set can0 down || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

---

## 2) File layout
- **Recommended:** use `canbase_patched.py` (this repo/canvas). It includes the ACK fix + `--debug`.
- Optionally rename it to `canbase.py` in your project.

Show help:
```bash
python canbase_patched.py -h
```
You should see subcommands:
```
ping, get-id, set-id, mo, jv, sp, pr, pa, bg, stop
```
Use `--channel can1` if your interface is not `can0`.

---

## 3) Quick Start

### 3.1 Safe "ping" (no motion)
Open a monitor in another terminal:
```bash
candump can0    # or: cansniffer can0
```
Send ping (Model Info / ML) to a few IDs:
```bash
python canbase_patched.py ping 6 7 8
```
If the node is alive, you’ll see an ACK with **CW=0x0B** and model/firmware payload.

### 3.2 Read current Node ID (PP[7])
```bash
# example from node 5
python canbase_patched.py get-id 5
```

### 3.3 Change Node ID (e.g., 5 → 8)
> Recommended to turn the motor **OFF** so params can be saved; `set-id` does this automatically and reboots.
```bash
python canbase_patched.py set-id 5 8
# verify on the new ID
python canbase_patched.py ping 8
```

---

## 4) Motor Control (safe & incremental)
> Start with small speeds.

### 4.1 Enable/disable the driver
```bash
python canbase_patched.py mo 8 on   # enable (current ON)
python canbase_patched.py mo 8 off  # disable (freewheel)
```

### 4.2 JOG (speed mode)
```bash
python canbase_patched.py jv 8 200  # set 200 pps
python canbase_patched.py bg 8      # apply & start
python canbase_patched.py stop 8    # smooth stop (uses SD)
```

### 4.3 PTP (position mode)
```bash
# move +3200 pulses relative
python canbase_patched.py pr 8 3200
# PTP speed 800 pps
python canbase_patched.py sp 8 800
# start
python canbase_patched.py bg 8
```
> **Note:** Motion parameters (JV/SP/PR/PA/AC/DC/…) **take effect after** `bg`.

---

## 5) Python API (embed in your script)
```python
from canbase_patched import UIM342CAN

u = UIM342CAN(channel="can0", debug=True)  # debug=True prints TX/RX

# ping (ML)
print("Ping:", u.ping_ml(8))

# read Node ID (PP[7])
print("NodeID:", u.get_node_id(8))

# enable → jog 300 pps → apply → stop → disable
u.mo(8, True)
u.jv(8, 300)
u.bg(8)
u.stop(8)
u.mo(8, False)

# PTP: +3200 pulses @ 800 pps
u.pr(8, 3200)
u.sp(8, 800)
u.bg(8)
```

---

## 6) Debug & ACK variants (important)
Some UIM342 gateways/firmware confirm ACK in **two possible ways**:
1) **CW in CAN‑ID low byte** (our initial assumption), or
2) **CW in `data[0]`** of the response.

Example from a real `candump`:
```
TX 0430008B   [0]                 # ML need‑ACK to Node 6
RX 06080000   [8]  0B 22 14 1A 09 CA 01 9B
# 0x0B (ML base CW) appears in data[0]
```
The patched helper accepts **both** styles. For diagnostics, run with `--debug`:
```bash
python canbase_patched.py --debug ping 6
```
You will see TX/RX frames with parsed fields.

---

## 7) One‑liner can‑utils (optional)
Ping ML (ID 8) without Python:
```bash
# ML need‑ACK → 0x8B; for Node ID 8 CAN‑ID = 0x0440008B; DLC 0
cansend can0 0440008B#
```
The response may carry CW in the CAN‑ID low byte **or** in `data[0]`.

---

## 8) FAQ & Tips
**Why is the minimum Node ID 5?**  
ID 0 is global/broadcast; 1–4 are reserved/producer. To avoid bit‑pattern collisions in the 29‑bit CAN‑ID, consumer nodes live in 5…126.

**What is ACK?**  
The host sends **CW|0x80** (Need‑ACK). The device replies with the **base CW** (without 0x80) and returns data.

**Payload endianness?**  
Little‑endian (LSB first) for multi‑byte integers (e.g., 32‑bit).

**Bitrate?**  
Match your device/gateway (common: 250k/500k/1M). Change via `ip link`.

**No response?**  
Check bitrate, wiring & polarity, common ground, 120 Ω terminators, and correct Node ID. Try pinging a likely ID (e.g., 5 or 8). `cansniffer` helps visualize frames.

---

## 9) Safety & Best Practices
- Do not hot‑plug CAN connectors under power.
- Start slow; ensure the mechanics are free of binding.
- Prefer **SD** (Stop Deceleration) for gentle stops.
- Save parameters only when **MO=0**; Node ID changes **take effect after reboot**.

---

## 10) Compact CW Reference
- **ML** 0x0B (ping/model), **SN** 0x0C, **ER** 0x0F
- **MO** 0x15, **BG** 0x16, **JV** 0x1D, **SP** 0x1E
- **PR** 0x1F, **PA** 0x20, **AC** 0x19, **DC** 0x1A, **SD** 0x1C
- **PP** 0x01 (PP[7] = Node ID), **SY** 0x7E (reboot, no‑ACK)

> Full details in the manual **Manual_UIM342AB V5.71** (Instruction Set).

---

## 11) License
Use freely in your projects. Please keep attribution if you redistribute.
