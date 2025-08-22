dia ada 3 macam id:
- broadcast id = 0
- group id
- node id

maksimal microstep adalah 128


# canbase — UIM342AB SimpleCAN3.0 Helper

Helper/utility untuk berkomunikasi dengan **UIROBOT UIM342AB** (servo stepper) lewat **SimpleCAN3.0** di atas **CAN 2.0B**. Berisi: encoder 29‑bit CAN‑ID, fungsi ping, baca/set Node ID, kontrol motor (MO/JV/SP/PR/PA/BG/STOP), serta contoh penggunaan via **CLI** dan **Python API**.

> **Catatan penting**
>
> * UIM342AB **tidak memakai CANopen**. Protokolnya adalah **SimpleCAN3.0** (proprietary) dengan payload little‑endian.
> * **Rentang Node ID valid: 5…126** (0 = global/broadcast; 1–4 reserved/producer).

---

## 1) Prasyarat

* Linux dengan **SocketCAN** (can-utils terinstal: `candump`, `cansend`, `cansniffer`).
* Python 3.8+ dan **python-can**.
* Jaringan CAN sudah benar (ground bersama, terminator **120 Ω** di kedua ujung bus, bitrate cocok).

### Instalasi python-can

```bash
pip install python-can
```

### Menyalakan SocketCAN

Ganti `500000` sesuai jaringanmu (125000/250000/500000/1000000).

```bash
sudo ip link set can0 down || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

---

## 2) Pasang `canbase.py`

Simpan file `canbase.py` (lihat di panel canvas) ke folder kerja. Cek bantuan:

```bash
python canbase.py -h
```

Kamu akan melihat subcommand:

```
ping, get-id, set-id, mo, jv, sp, pr, pa, bg, stop
```

Tambahkan `--channel can1` kalau antarmuka kamu bukan `can0`.

---

## 3) Quick Start

### 3.1 Cek balasan ("ping" aman, tanpa gerak)

Buka terminal pemantau:

```bash
candump can0    # atau: cansniffer can0
```

Kirim ping ke beberapa ID (Model Info / ML):

```bash
python canbase.py ping 6 7 8
```

Jika node hidup, kamu akan melihat ACK CW=0x0B dengan payload model/firmware.

### 3.2 Baca Node ID saat ini (PP\[7])

```bash
# contoh dari node 5
python canbase.py get-id 5
```

### 3.3 Ganti Node ID (misal 5 → 8)

> Disarankan **motor OFF** dahulu agar parameter tersimpan; `set-id` melakukan ini otomatis lalu reboot.

```bash
python canbase.py set-id 5 8
# kemudian verifikasi
python canbase.py ping 8
```

---

## 4) Kontrol Motor (aman & bertahap)

> Rekomendasi: mulai dari kecepatan kecil.

### 4.1 Enable/disable driver

```bash
python canbase.py mo 8 on   # enable driver (arus ON)
python canbase.py mo 8 off  # disable (freewheel)
```

### 4.2 JOG (speed mode)

```bash
python canbase.py jv 8 200  # set 200 pps
python canbase.py bg 8      # apply & mulai jalan
python canbase.py stop 8    # berhenti halus (pakai SD)
```

### 4.3 PTP (position mode)

```bash
# gerak relatif +3200 pulse
python canbase.py pr 8 3200
# speed PTP 800 pps
python canbase.py sp 8 800
# mulai
python canbase.py bg 8
```

> **Catatan:** Parameter gerak (JV/SP/PR/PA/AC/DC/…) **baru aktif** setelah `bg`.

---

## 5) Python API (langsung di script)

```python
from canbase import UIM342CAN

# buka can0
u = UIM342CAN(channel="can0")

# ping (ML)
print("Ping:", u.ping_ml(8))

# baca Node ID (PP[7])
print("NodeID:", u.get_node_id(8))

# enable → jog 300 pps → apply → stop → disable
u.mo(8, True)
u.jv(8, 300)
u.bg(8)
u.stop(8)
u.mo(8, False)

# PTP: +3200 pulse @ 800 pps
u.pr(8, 3200)
u.sp(8, 800)
u.bg(8)
```

---

## 6) FAQ & Tips

**Q: Kenapa Node ID minimal 5?**
A: ID 0 dipakai global/broadcast, 1–4 reserved/producer. Untuk hindari tabrakan pola bit di 29‑bit CAN‑ID, node/consumer ditaruh di 5…126.

**Q: ACK itu apa?**
Host mengirim **CW|0x80** (Need‑ACK). Device membalas dengan **CW dasar** (tanpa 0x80) dan data.

**Q: Endian payload?**
Little‑endian (LSB dulu) untuk bilangan multi‑byte (contoh nilai 32‑bit).

**Q: Bitrate?**
Cocokkan dengan perangkat/gateway (umum: 250k/500k/1M). Ubah via `ip link`.

**Q: Tidak ada balasan?**
Cek: bitrate, kabel & polaritas, ground bersama, terminator 120 Ω, Node ID benar. Coba ping satu ID yang pasti (mis. 5 atau 8). Pakai `cansniffer` untuk melihat frame masuk.

---

## 7) Keamanan & Best Practice

* Jangan hot‑plug konektor CAN saat bertegangan.
* Mulai dari kecepatan rendah; perhatikan mekanik bebas hambatan.
* Gunakan **SD** (Stop Deceleration) agar berhenti halus.
* Simpan parameter hanya saat **MO=0**; perubahan Node ID **aktif setelah reboot**.

---

## 8) Referensi CW (ringkas)

* **ML** 0x0B (ping/model), **SN** 0x0C, **ER** 0x0F
* **MO** 0x15, **BG** 0x16, **JV** 0x1D, **SP** 0x1E
* **PR** 0x1F, **PA** 0x20, **AC** 0x19, **DC** 0x1A, **SD** 0x1C
* **PP** 0x01 (PP\[7] = Node ID), **SY** 0x7E (reboot, no‑ACK)

> Detail lengkap ada di manual **UIM342AB V5.71** (Instruction Set).

---

## 9) Contoh satu-baris can‑utils (opsional)

Ping ML (ID 8), langsung dengan can‑utils (tanpa Python):

```bash
# CW ML need‑ACK → 0x8B; untuk ID 8 CAN‑ID = 0x0440008B; DLC 0
cansend can0 0440008B#
```

> Balasan akan tampak dengan CW 0x0B dan 8‑byte payload model/firmware.

---

## 10) Lisensi

Gunakan bebas untuk proyekmu. Mohon sertakan kredit jika di-redistribusikan.
