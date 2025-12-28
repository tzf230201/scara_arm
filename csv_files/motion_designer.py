import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os, json, subprocess
import pandas as pd
import math

# === Flag Global ===
PP_MOTION_ENABLE = False  # kalau False, PP_Coor & PP_Angle tidak muncul


# --- Utility: baca baris pertama CSV ---
def read_first_xyz_yaw(csv_path):
    """Baca baris pertama dari file CSV dan ambil X, Y, Z, Yaw (jika ada)."""
    try:
        df = pd.read_csv(csv_path)
        if df.empty:
            return None

        # map kolom secara case-insensitive
        cols = {str(c).strip().lower(): c for c in df.columns}

        def get_first(*names, default=0.0):
            for name in names:
                key = str(name).strip().lower()
                if key in cols:
                    return float(df.at[0, cols[key]])
            return float(default)

        # dukung X/Y/Z/C atau X/Y/Z/Yaw
        x = get_first("x")
        y = get_first("y")
        z = get_first("z")
        yaw_or_c = get_first("c", "yaw")
        return x, y, z, yaw_or_c
    except Exception:
        return None


def read_last_xyz_yaw(csv_path):
    """Baca baris terakhir dari file CSV dan ambil X, Y, Z, Yaw/C (jika ada)."""
    try:
        df = pd.read_csv(csv_path)
        if df.empty:
            return None

        cols = {str(c).strip().lower(): c for c in df.columns}

        def get_last(*names, default=0.0):
            for name in names:
                key = str(name).strip().lower()
                if key in cols:
                    return float(df.iloc[-1][cols[key]])
            return float(default)

        x = get_last("x")
        y = get_last("y")
        z = get_last("z")
        yaw_or_c = get_last("c", "yaw")
        return x, y, z, yaw_or_c
    except Exception:
        return None


def parse_details(details_str):
    """Parse 'a=1, b=2' jadi dict (tahan spasi)."""
    if not details_str:
        return {}
    parts = [p.strip() for p in str(details_str).split(",") if p.strip()]
    out = {}
    for p in parts:
        if "=" in p:
            k, v = p.split("=", 1)
            out[k.strip().lower()] = v.strip()
    return out


def servo_forward_kinematics(angle_1):
    # sama seperti pvt_generator.py
    return (90.0 / 360.0) * float(angle_1)


def arm_forward_kinematics(angle_2, angle_3, angle_4):
    # sama seperti pvt_generator.py
    angle_4 = float(angle_4) * -1.0
    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5

    theta2_rad = math.radians((float(angle_2) / 5.0) + OFFSET_2)
    theta3_rad = math.radians((float(angle_3) / 5.0) + OFFSET_3 - (float(angle_2) / 5.0))

    x2 = L2 * math.cos(theta2_rad)
    y2 = L2 * math.sin(theta2_rad)
    x3 = x2 + L3 * math.cos(theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta2_rad + theta3_rad)
    yaw = (angle_4 / 5.0) + OFFSET_4

    return x3, y3, yaw


def estimate_pose_from_row(mtype, details_str, base_dir):
    """Estimasi pose akhir motion: return (x, y, z, c/yaw) atau None jika tidak bisa."""
    d = parse_details(details_str)
    try:
        if mtype == "pvt_coor":
            x = float(d.get("x", 0))
            y = float(d.get("y", 0))
            z = float(d.get("z", 0))
            c = float(d.get("c", d.get("yaw", 0)))
            return x, y, z, c

        if mtype == "pvt_angle":
            a1 = float(d.get("angle_1", 0))
            a2 = float(d.get("angle_2", 0))
            a3 = float(d.get("angle_3", 0))
            a4 = float(d.get("angle_4", 0))
            z = servo_forward_kinematics(a1)
            x, y, c = arm_forward_kinematics(a2, a3, a4)
            return x, y, z, c

        if mtype == "pvt_csv":
            csv_name = d.get("csv")
            if not csv_name:
                return None
            csv_path = os.path.join(base_dir, csv_name)
            last = read_last_xyz_yaw(csv_path)
            if last is None:
                return None
            x, y, z, c = last
            try:
                offset_z = float(d.get("offset_z", 0) or 0)
            except Exception:
                offset_z = 0.0
            return x, y, z + offset_z, c
    except Exception:
        return None

    return None


# --- Popup untuk tambah/edit motion ---
class MotionPopup(tk.Toplevel):
    def __init__(self, master, folder=".", on_submit=None,
                 current_count=0, max_repeat_index=None,
                 motion_type=None, initial_data=None, edit_index=None):
        super().__init__(master)
        self.title("Edit Motion" if edit_index is not None else "Add Motion")
        self.geometry("460x740")

        self.on_submit = on_submit
        self.folder = folder
        self.current_count = current_count
        self.max_repeat_index = max_repeat_index if max_repeat_index is not None else current_count
        self.edit_index = edit_index

        # default type
        default_type = "pvt_coor"
        if PP_MOTION_ENABLE:
            default_type = "pp_coor"

        self.motion_type = tk.StringVar(value=motion_type or default_type)

        # === Daftar motion type ===
        types = []
        if PP_MOTION_ENABLE:
            types.extend([("pp_coor", "pp_coor"), ("pp_angle", "pp_angle")])
        types.extend([
            ("pvt_coor", "pvt_coor"),
            ("pvt_angle", "pvt_angle"),
            ("pvt_csv", "pvt_csv"),
            ("pause", "pause"),
            ("repeat", "repeat"),
        ])

        tk.Label(self, text="Pilih Motion Type:").pack(anchor="w", padx=10, pady=(10, 2))
        for txt, val in types:
            tk.Radiobutton(self, text=txt, variable=self.motion_type, value=val,
                           command=lambda: self.update_inputs(initial_data)).pack(anchor="w", padx=12)

        # Frame input dinamis
        self.input_frame = tk.Frame(self)
        self.input_frame.pack(fill="both", expand=True, padx=10, pady=10)
        self.update_inputs(initial_data)

        # Tombol OK & Cancel
        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=8)
        tk.Button(btn_frame, text="OK", width=12, command=self.submit).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Cancel", width=12, command=self.destroy).pack(side="left", padx=5)

        # shortcuts
        self.bind("<Escape>", lambda e: self.destroy())
        self.bind("<Return>", lambda e: self.submit())

    # === Update isi field ===
    def update_inputs(self, initial_data=None):
        for w in self.input_frame.winfo_children():
            w.destroy()
        self.entries = {}
        motion = self.motion_type.get()

        def init_value(key, default):
            if initial_data and key in initial_data:
                return initial_data[key]
            return default

        # --- helper widgets ---
        def label_entry(k, val):
            tk.Label(self.input_frame, text=f"{k}:").pack(anchor="w")
            ent = tk.Entry(self.input_frame)
            ent.insert(0, str(val))
            ent.pack(fill="x", pady=2)
            self.entries[k] = ent
            return ent

        # --- PVT_Coor ---
        if motion in ["pvt_coor"]:
            defaults = {"z": 90, "x": 258, "y": 0, "c": 0, "t_arm": 4000, "t_servo": 4000}
            # urutan z, x, y, c (sesuai permintaan sebelumnya)
            for k in ["z", "x", "y", "c"]:
                label_entry(k, init_value(k, defaults[k]))

            # t_arm dan auto-sync ke t_servo
            t_arm_entry = label_entry("t_arm", init_value("t_arm", defaults["t_arm"]))
            t_arm_entry.bind("<KeyRelease>", lambda e: self.sync_t_servo())
            self.entries["t_arm"] = t_arm_entry

            # t_servo (hidden kalau checkbox aktif)
            self.t_servo_frame = tk.Frame(self.input_frame)
            tk.Label(self.t_servo_frame, text="t_servo:").pack(anchor="w")
            self.t_servo_entry = tk.Entry(self.t_servo_frame)
            self.t_servo_entry.insert(0, str(init_value("t_servo", defaults["t_servo"])))
            self.t_servo_entry.pack(fill="x", pady=2)
            self.entries["t_servo"] = self.t_servo_entry

            # checkbox sync
            equal_default = True
            if initial_data:
                try:
                    equal_default = (str(initial_data.get("t_arm", "")) == str(initial_data.get("t_servo", "")))
                except Exception:
                    equal_default = True
            self.t_arm_equal = tk.BooleanVar(value=equal_default)
            chk = tk.Checkbutton(self.input_frame, text="t_arm = t_servo",
                                 variable=self.t_arm_equal, command=self.toggle_t_servo)
            chk.pack(anchor="w", pady=5)
            self.entries["t_arm_equal"] = self.t_arm_equal

            # tampilkan/hidden t_servo sesuai checkbox
            self.toggle_t_servo()

        # --- PVT_Angle ---
        elif motion in ["pvt_angle"]:
            defaults = {"angle_1": 360, "angle_2": 0, "angle_3": 0, "angle_4": 0,
                        "t_arm": 4000, "t_servo": 4000}
            for k in ["angle_1", "angle_2", "angle_3", "angle_4"]:
                label_entry(k, init_value(k, defaults[k]))

            t_arm_entry = label_entry("t_arm", init_value("t_arm", defaults["t_arm"]))
            t_arm_entry.bind("<KeyRelease>", lambda e: self.sync_t_servo())
            self.entries["t_arm"] = t_arm_entry

            self.t_servo_frame = tk.Frame(self.input_frame)
            tk.Label(self.t_servo_frame, text="t_servo:").pack(anchor="w")
            self.t_servo_entry = tk.Entry(self.t_servo_frame)
            self.t_servo_entry.insert(0, str(init_value("t_servo", defaults["t_servo"])))
            self.t_servo_entry.pack(fill="x", pady=2)
            self.entries["t_servo"] = self.t_servo_entry

            equal_default = True
            if initial_data:
                try:
                    equal_default = (str(initial_data.get("t_arm", "")) == str(initial_data.get("t_servo", "")))
                except Exception:
                    equal_default = True
            self.t_arm_equal = tk.BooleanVar(value=equal_default)
            chk = tk.Checkbutton(self.input_frame, text="t_arm = t_servo",
                                 variable=self.t_arm_equal, command=self.toggle_t_servo)
            chk.pack(anchor="w", pady=5)
            self.entries["t_arm_equal"] = self.t_arm_equal

            self.toggle_t_servo()

        # --- PVT_CSV ---
        elif motion == "pvt_csv":
            tk.Label(self.input_frame, text="CSV File:").pack(anchor="w")
            files = [f for f in os.listdir(self.folder) if f.lower().endswith(".csv")]
            self.entries["csv"] = ttk.Combobox(self.input_frame, values=files, state="readonly")
            self.entries["csv"].pack(fill="x", pady=2)
            if files:
                self.entries["csv"].set(init_value("csv", files[0]))

            v_max_default, acc_default, offset_z_default = 100, 2000, 0
            # v_safe default = v_max/10
            v_max_entry = label_entry("v_max", init_value("v_max", v_max_default))
            v_max_entry.bind("<KeyRelease>", lambda e: self.sync_vsafe())
            self.entries["v_max"] = v_max_entry
            label_entry("acc_dec", init_value("acc_dec", acc_default))
            label_entry("offset_z", init_value("offset_z", offset_z_default))

            # v_safe field (hidden bila autoset aktif)
            self.vsafe_frame = tk.Frame(self.input_frame)
            tk.Label(self.vsafe_frame, text="v_safe:").pack(anchor="w")
            self.vsafe_entry = tk.Entry(self.vsafe_frame)

            # tentukan default autoset dari initial (jika edit)
            autoset_default = True
            init_vmax = init_value("v_max", v_max_default)
            init_vsafe = init_value("v_safe", round(float(init_vmax) / 10))
            try:
                autoset_default = (int(round(float(init_vsafe))) == int(round(float(init_vmax) / 10)))
            except Exception:
                autoset_default = True
            self.vsafe_entry.insert(0, str(init_vsafe))
            self.vsafe_entry.pack(fill="x", pady=2)
            self.entries["v_safe"] = self.vsafe_entry

            self.autoset_vsafe = tk.BooleanVar(value=autoset_default)
            chk = tk.Checkbutton(self.input_frame, text="Autoset v_safe by system",
                                 variable=self.autoset_vsafe, command=self.toggle_vsafe)
            chk.pack(anchor="w", pady=5)
            self.entries["Autoset_vsafe"] = self.autoset_vsafe

            # tampilkan/hidden v_safe sesuai autoset
            self.toggle_vsafe()
            # sync awal bila autoset true
            self.sync_vsafe()

        # --- Pause ---
        elif motion == "pause":
            label_entry("t_wait", init_value("t_wait", 1000))

        # --- Repeat ---
        elif motion == "repeat":
            if self.current_count < 1:
                tk.Label(self.input_frame, text="List kosong (butuh ≥1 motion)").pack(anchor="w")
                return

            max_idx = max(1, self.max_repeat_index)

            # from_index
            tk.Label(self.input_frame, text="from_index:").pack(anchor="w")
            self.entries["from_index"] = ttk.Combobox(
                self.input_frame, state="readonly",
                values=list(range(1, max_idx + 1))
            )
            self.entries["from_index"].pack(fill="x", pady=2)
            self.entries["from_index"].set(str(init_value("from_index", 1)))

            # to_index
            tk.Label(self.input_frame, text="to_index:").pack(anchor="w")
            self.entries["to_index"] = ttk.Combobox(
                self.input_frame, state="readonly",
                values=list(range(1, max_idx + 1))
            )
            self.entries["to_index"].pack(fill="x", pady=2)
            self.entries["to_index"].set(str(init_value("to_index", min(1, max_idx))))

            def update_to_index(_e=None):
                try:
                    fval = int(self.entries["from_index"].get())
                except:
                    fval = 1
                self.entries["to_index"]["values"] = list(range(fval, max_idx + 1))
                cur = int(self.entries["to_index"].get() or fval)
                if cur < fval:
                    self.entries["to_index"].set(str(fval))

            self.entries["from_index"].bind("<<ComboboxSelected>>", update_to_index)
            update_to_index()

            # how_many_times
            label_entry("how_many_times", init_value("how_many_times", 1))

    def toggle_t_servo(self):
        # show/hide t_servo frame berdasarkan checkbox
        if hasattr(self, "t_arm_equal") and self.t_arm_equal.get():
            # samakan nilai & hide
            self.sync_t_servo()
            if hasattr(self, "t_servo_frame"):
                self.t_servo_frame.pack_forget()
        else:
            if hasattr(self, "t_servo_frame"):
                self.t_servo_frame.pack(fill="x", pady=2)

    def toggle_vsafe(self):
        if hasattr(self, "autoset_vsafe") and self.autoset_vsafe.get():
            # hide v_safe & sync dari v_max
            if hasattr(self, "vsafe_frame"):
                self.vsafe_frame.pack_forget()
            self.sync_vsafe()
        else:
            if hasattr(self, "vsafe_frame"):
                self.vsafe_frame.pack(fill="x", pady=2)

    def sync_t_servo(self):
        if hasattr(self, "t_arm_equal") and self.t_arm_equal.get():
            try:
                val = self.entries["t_arm"].get()
                self.entries["t_servo"].delete(0, tk.END)
                self.entries["t_servo"].insert(0, val)
            except Exception:
                pass

    def sync_vsafe(self):
        if hasattr(self, "autoset_vsafe") and self.autoset_vsafe.get():
            try:
                vmax = float(self.entries["v_max"].get())
                self.entries["v_safe"].delete(0, tk.END)
                self.entries["v_safe"].insert(0, str(int(round(vmax / 10))))
            except Exception:
                # biarkan apa adanya bila parsing gagal
                pass

    def submit(self):
        motion = self.motion_type.get()
        # kumpulkan data
        data = {}
        for k, v in self.entries.items():
            if isinstance(v, tk.BooleanVar):
                data[k] = v.get()
            elif hasattr(v, "get"):
                data[k] = v.get()
            else:
                data[k] = v

        # bersihkan flag internal (tetap mempertahankan efeknya)
        data.pop("t_arm_equal", None)
        data.pop("Autoset_vsafe", None)

        if self.on_submit:
            self.on_submit(motion, data, self.edit_index)
        self.destroy()


# --- GUI utama MotionDesigner ---
class MotionDesigner:
    def __init__(self, root):
        self.root = root
        self.root.title("Motion Designer")
        self.loaded_json = None  # track file json yang sedang dibuka

        # gunakan folder script agar path CSV/pvt_generator konsisten walau cwd berubah
        self.base_dir = os.path.dirname(os.path.abspath(__file__))

        self.tree = ttk.Treeview(root, columns=("Index", "Type", "Details"), show="headings")
        for h in ["Index", "Type", "Details"]:
            self.tree.heading(h, text=h)
            self.tree.column(h, anchor="center")
        self.tree.pack(fill="both", expand=True, padx=10, pady=10)

        frame = tk.Frame(root)
        frame.pack(pady=5)
        tk.Button(frame, text="Add Motion", command=self.add_motion).pack(side="left", padx=5)
        tk.Button(frame, text="Edit Motion", command=self.edit_motion).pack(side="left", padx=5)
        tk.Button(frame, text="Remove Motion", command=self.remove_motion).pack(side="left", padx=5)
        tk.Button(frame, text="Save", command=self.save_motions).pack(side="left", padx=5)
        tk.Button(frame, text="Load", command=self.load_motions).pack(side="left", padx=5)
        tk.Button(frame, text="Export G-code", command=self.export_gcode).pack(side="left", padx=5)

    def autosize_columns(self):
        import tkinter.font as tkFont
        for col in self.tree["columns"]:
            font = tkFont.Font()
            max_width = font.measure(col) + 24
            for item in self.tree.get_children():
                width = font.measure(str(self.tree.set(item, col))) + 24
                max_width = max(max_width, width)
            self.tree.column(col, width=max_width)

    def add_motion(self):
        # hitung batas repeat hanya sampai index terpilih (insert setelahnya)
        children = list(self.tree.get_children())
        selected = self.tree.selection()
        if selected:
            sel_1based = children.index(selected[0]) + 1
            max_repeat_index = sel_1based
        else:
            max_repeat_index = len(children)

        MotionPopup(
            self.root,
            folder=self.base_dir,
            on_submit=self.insert_motion,
            current_count=len(children),
            max_repeat_index=max_repeat_index
        )

    def edit_motion(self):
        sel = self.tree.selection()
        if not sel:
            messagebox.showwarning("Warning", "Please select a motion to edit.")
            return
        item = sel[0]
        _idx, mtype, details = self.tree.item(item, "values")

        # parse details "a=1, b=2"
        initial_data = {}
        if details.strip():
            parts = [p.strip() for p in details.split(",")]
            for p in parts:
                if "=" in p:
                    k, v = p.split("=", 1)
                    initial_data[k.strip()] = v.strip()

        # sediakan max_repeat_index berdasar posisi item (agar tidak bisa repeat melewati baris setelahnya saat insert)
        children = list(self.tree.get_children())
        sel_1based = children.index(item) + 1

        MotionPopup(
            self.root,
            folder=self.base_dir,
            on_submit=self.insert_motion,
            current_count=len(children),
            max_repeat_index=sel_1based,
            motion_type=mtype,
            initial_data=initial_data,
            edit_index=item
        )

    def insert_motion(self, motion_type, data, edit_index=None):
        # normalisasi key ke lowercase
        normalized = {k.lower(): v for k, v in data.items()}
        # Jika motion coor/angle & checkbox aktif (kita sudah sync di popup), cukup pakai nilai yang ada
        details = ", ".join(f"{k}={v}" for k, v in normalized.items())

        children = list(self.tree.get_children())
        selected = self.tree.selection()

        if edit_index:
            # update row yang diedit (Index dipertahankan)
            idx, _, _ = self.tree.item(edit_index, "values")
            self.tree.item(edit_index, values=(idx, motion_type, details))
            new_item = edit_index
        else:
            # insert setelah row terpilih; kalau tidak ada, append di akhir
            if selected:
                sel_pos = children.index(selected[0])
                insert_pos = sel_pos + 1
            else:
                insert_pos = "end"

            # khusus pvt_csv: tambahkan pvt_coor dulu (target = baris pertama CSV)
            if motion_type == "pvt_csv":
                csv_name = normalized.get("csv")
                pre_item = None

                if csv_name:
                    csv_path = os.path.join(self.base_dir, csv_name)
                    first = read_first_xyz_yaw(csv_path)
                else:
                    first = None

                if first is None:
                    messagebox.showwarning(
                        "Warning",
                        "Tidak bisa membaca baris pertama CSV untuk membuat pvt_coor.\n"
                        "Motion pvt_csv akan tetap ditambahkan tanpa pre-motion pvt_coor."
                    )
                    new_item = self.tree.insert("", insert_pos, values=("", motion_type, details))
                else:
                    x, y, z, c = first
                    try:
                        offset_z = float(normalized.get("offset_z", 0) or 0)
                    except Exception:
                        offset_z = 0.0
                    z = z + offset_z

                    # hitung timing berdasarkan jarak dari pose sebelumnya dan v_max (mm/s)
                    prev_pose = None
                    if selected:
                        prev_item = selected[0]
                    else:
                        prev_item = children[-1] if len(children) > 0 else None

                    if prev_item is not None:
                        _pidx, ptype, pdetails = self.tree.item(prev_item, "values")
                        prev_pose = estimate_pose_from_row(ptype, pdetails, self.base_dir)

                    # fallback jika belum bisa estimasi pose sebelumnya
                    if prev_pose is None:
                        prev_pose = (0.0, 0.0, 0.0, 0.0)

                    try:
                        v_max = float(normalized.get("v_max", 0) or 0)
                    except Exception:
                        v_max = 0.0

                    dx = float(x) - float(prev_pose[0])
                    dy = float(y) - float(prev_pose[1])
                    dz = float(z) - float(prev_pose[2])
                    dist_mm = math.sqrt(dx * dx + dy * dy + dz * dz)

                    # t (ms) = dist(mm) / v(mm/s) * 1000
                    if v_max <= 0:
                        t_ms = 100
                    else:
                        t_ms = int(round((dist_mm / v_max) * 1000.0))
                        if t_ms < 100:
                            t_ms = 100

                    pre_details_ordered = [
                        ("z", z),
                        ("x", x),
                        ("y", y),
                        ("c", c),
                        ("t_arm", t_ms),
                        ("t_servo", t_ms),
                    ]
                    pre_details = ", ".join(f"{k}={v}" for k, v in pre_details_ordered)

                    pre_item = self.tree.insert("", insert_pos, values=("", "pvt_coor", pre_details))
                    if isinstance(insert_pos, int):
                        insert_pos_csv = insert_pos + 1
                    else:
                        insert_pos_csv = "end"
                    new_item = self.tree.insert("", insert_pos_csv, values=("", motion_type, details))
            else:
                new_item = self.tree.insert("", insert_pos, values=("", motion_type, details))

        # reindex ulang 1..N
        for i, item in enumerate(self.tree.get_children(), start=1):
            vals = list(self.tree.item(item, "values"))
            vals[0] = i
            self.tree.item(item, values=vals)

        # select & scroll ke item baru / yang diedit
        self.tree.selection_set(new_item)
        self.tree.focus(new_item)
        self.tree.see(new_item)

        self.autosize_columns()

    def remove_motion(self):
        for sel in self.tree.selection():
            self.tree.delete(sel)
        # reindex
        for i, item in enumerate(self.tree.get_children(), start=1):
            vals = list(self.tree.item(item, "values"))
            vals[0] = i
            self.tree.item(item, values=vals)
        self.autosize_columns()

    def save_motions(self):
        motions = [{"Index": v[0], "Type": v[1], "Details": v[2]} for v in
                   (self.tree.item(i, "values") for i in self.tree.get_children())]
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON files", "*.json")],
                                            title="Save Motions")
        if path:
            try:
                with open(path, "w") as f:
                    json.dump(motions, f, indent=2)
                self.loaded_json = os.path.basename(path)
                messagebox.showinfo("Success", f"Saved to {path}")
            except Exception as e:
                messagebox.showerror("Error", f"Gagal save: {e}")

    def load_motions(self):
        path = filedialog.askopenfilename(defaultextension=".json",
                                          filetypes=[("JSON files", "*.json")],
                                          title="Load Motions")
        if path and os.path.exists(path):
            try:
                with open(path, "r") as f:
                    motions = json.load(f)
                self.loaded_json = os.path.basename(path)
                # clear dulu
                for s in self.tree.get_children():
                    self.tree.delete(s)
                # isi
                for i, m in enumerate(motions, start=1):
                    self.tree.insert("", "end", values=(i, m["Type"], m["Details"]))
                self.autosize_columns()
                messagebox.showinfo("Success", f"Loaded {len(motions)} motions")
            except Exception as e:
                messagebox.showerror("Error", f"Gagal load: {e}")

    # === Export G-code (dengan opsi preview) ===
    def export_gcode(self):
        if not self.loaded_json:
            messagebox.showwarning("Warning", "Please load or save a JSON file first.")
            return

        answer = messagebox.askyesno("Preview", "Do you want to preview the motion?")
        preview_flag = "1" if answer else "0"
        cmd = ["python3", "pvt_generator.py", "--json", self.loaded_json, "--preview", preview_flag]

        try:
            subprocess.Popen(cmd, cwd=self.base_dir)
            messagebox.showinfo("Running", f"Executed:\n{' '.join(cmd)}")
        except Exception as e:
            messagebox.showerror("Error", f"Cannot execute:\n{' '.join(cmd)}\n\n{e}")


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1100x600")
    root.minsize(900, 480)
    MotionDesigner(root)
    root.mainloop()
