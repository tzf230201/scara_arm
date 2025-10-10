import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os, json
import pandas as pd  # Untuk membaca CSV

# === Flag Global ===
PP_MOTION_ENABLE = False  # kalau False, PP_Coor & PP_Angle tidak muncul


# --- Utility: baca baris pertama CSV ---
def read_first_xyz_yaw(csv_path):
    """Baca baris pertama dari file CSV dan ambil X, Y, Z, Yaw (jika ada)."""
    try:
        df = pd.read_csv(csv_path)
        cols = {c.lower(): c for c in df.columns}
        x = float(df[cols.get("x", 0)][0]) if "x" in cols else 0
        y = float(df[cols.get("y", 0)][0]) if "y" in cols else 0
        z = float(df[cols.get("z", 0)][0]) if "z" in cols else 0
        yaw = float(df[cols.get("yaw", 0)][0]) if "yaw" in cols else 0
        return x, y, z, yaw
    except Exception:
        return None


# --- Popup untuk tambah motion ---
class MotionPopup(tk.Toplevel):
    def __init__(self, master, folder=".", on_submit=None, current_count=0, max_repeat_index=None):
        super().__init__(master)
        self.title("Add Motion")
        self.geometry("420x680")

        self.on_submit = on_submit
        self.folder = folder
        self.current_count = current_count
        self.max_repeat_index = max_repeat_index if max_repeat_index is not None else current_count

        if PP_MOTION_ENABLE:
            default_type = "pp_coor"
        else:
            default_type = "pvt_coor"

        self.motion_type = tk.StringVar(value=default_type)

        # === Daftar motion type ===
        types = []
        if PP_MOTION_ENABLE:
            types.extend([
                ("pp_coor", "pp_coor"),
                ("pp_angle", "pp_angle"),
            ])
        types.extend([
            ("pvt_coor", "pvt_coor"),
            ("pvt_angle", "pvt_angle"),
            ("pvt_csv", "pvt_csv"),
            ("pause", "pause"),
            ("repeat", "repeat"),
        ])

        tk.Label(self, text="Pilih Motion Type:").pack(anchor="w")
        for txt, val in types:
            tk.Radiobutton(
                self, text=txt, variable=self.motion_type, value=val,
                command=self.update_inputs
            ).pack(anchor="w")

        # Frame input dinamis
        self.input_frame = tk.Frame(self)
        self.input_frame.pack(fill="both", expand=True, pady=10)
        self.update_inputs()

        # Tombol OK & Cancel
        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=5)
        tk.Button(btn_frame, text="OK", command=self.submit).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Cancel", command=self.destroy).pack(side="left", padx=5)

        self.bind("<Escape>", lambda e: self.destroy())
        self.bind("<Return>", lambda e: self.submit())

    # === Update field input sesuai tipe motion ===
    def update_inputs(self):
        for w in self.input_frame.winfo_children():
            w.destroy()
        self.entries = {}
        motion = self.motion_type.get()

        # --- PP_Coor & PVT_Coor ---
        if motion in ["pp_coor", "pvt_coor"]:
            defaults = {"z": 90, "x": 258, "y": 0, "c": 0, "t_arm": 4000, "t_servo": 4000}
            for label in ["z", "x", "y", "c", "t_arm"]:
                self.add_entry(label, defaults[label])

            self.t_servo_frame = tk.Frame(self.input_frame)
            tk.Label(self.t_servo_frame, text="t_servo:").pack(anchor="w")
            self.t_servo_entry = tk.Entry(self.t_servo_frame)
            self.t_servo_entry.insert(0, str(defaults["t_servo"]))
            self.t_servo_entry.pack(fill="x", pady=2)
            self.entries["t_servo"] = self.t_servo_entry
            self.t_servo_frame.pack_forget()

            self.t_arm_equal = tk.BooleanVar(value=True)
            chk = tk.Checkbutton(self.input_frame, text="t_arm = t_servo",
                                 variable=self.t_arm_equal, command=self.toggle_t_servo)
            chk.pack(anchor="w", pady=5)
            self.entries["t_arm_equal"] = self.t_arm_equal

        # --- PP_Angle & PVT_Angle ---
        elif motion in ["pp_angle", "pvt_angle"]:
            defaults = {"angle_1": 360, "angle_2": 0, "angle_3": 0, "angle_4": 0,
                        "t_arm": 4000, "t_servo": 4000}
            for label in ["angle_1", "angle_2", "angle_3", "angle_4", "t_arm"]:
                self.add_entry(label, defaults[label])

            self.t_servo_frame = tk.Frame(self.input_frame)
            tk.Label(self.t_servo_frame, text="t_servo:").pack(anchor="w")
            self.t_servo_entry = tk.Entry(self.t_servo_frame)
            self.t_servo_entry.insert(0, str(defaults["t_servo"]))
            self.t_servo_entry.pack(fill="x", pady=2)
            self.entries["t_servo"] = self.t_servo_entry
            self.t_servo_frame.pack_forget()

            self.t_arm_equal = tk.BooleanVar(value=True)
            chk = tk.Checkbutton(self.input_frame, text="t_arm = t_servo",
                                 variable=self.t_arm_equal, command=self.toggle_t_servo)
            chk.pack(anchor="w", pady=5)
            self.entries["t_arm_equal"] = self.t_arm_equal

        # --- PVT_CSV ---
        elif motion == "pvt_csv":
            tk.Label(self.input_frame, text="CSV File:").pack(anchor="w")
            files = [f for f in os.listdir(self.folder) if f.endswith(".csv")]
            self.entries["csv"] = ttk.Combobox(self.input_frame, values=files, state="readonly")
            self.entries["csv"].pack(fill="x", pady=2)
            if files:
                self.entries["csv"].current(0)

            v_max_default, acc_default, offset_z_default = 100, 2000, 0
            v_safe_default = v_max_default / 10

            self.add_entry("v_max", v_max_default)
            self.add_entry("acc_dec", acc_default)
            self.add_entry("offset_z", offset_z_default)

            self.vsafe_frame = tk.Frame(self.input_frame)
            tk.Label(self.vsafe_frame, text="v_safe:").pack(anchor="w")
            self.vsafe_entry = tk.Entry(self.vsafe_frame)
            self.vsafe_entry.insert(0, str(v_safe_default))
            self.vsafe_entry.pack(fill="x", pady=2)
            self.entries["v_safe"] = self.vsafe_entry
            self.vsafe_frame.pack_forget()

            self.autoset_vsafe = tk.BooleanVar(value=True)
            chk = tk.Checkbutton(self.input_frame, text="Autoset v_safe by system",
                                 variable=self.autoset_vsafe, command=self.toggle_vsafe)
            chk.pack(anchor="w", pady=5)
            self.entries["Autoset_vsafe"] = self.autoset_vsafe

        # --- Pause ---
        elif motion == "pause":
            self.add_entry("t_wait", 1000)

        # --- Repeat ---
        elif motion == "repeat":
            if self.current_count < 1:
                tk.Label(self.input_frame, text="list is empty (need ≥1 motion)").pack(anchor="w")
                return

            max_idx = max(1, self.max_repeat_index)
            tk.Label(self.input_frame, text="from_index:").pack(anchor="w")
            from_cb = ttk.Combobox(self.input_frame,
                                   values=list(range(1, max_idx + 1)), state="readonly")
            from_cb.set(1)
            from_cb.pack(fill="x", pady=2)
            self.entries["from_index"] = from_cb

            tk.Label(self.input_frame, text="to_index:").pack(anchor="w")
            to_cb = ttk.Combobox(self.input_frame,
                                 values=list(range(1, max_idx + 1)), state="readonly")
            to_cb.set(1)
            to_cb.pack(fill="x", pady=2)
            self.entries["to_index"] = to_cb

            def update_to_index(event):
                try:
                    fval = int(from_cb.get())
                except:
                    fval = 1
                to_cb["values"] = list(range(fval, max_idx + 1))
                to_cb.set(fval)
            from_cb.bind("<<ComboboxSelected>>", update_to_index)
            self.add_entry("how_many_times", 1)

    def add_entry(self, label, default_val=None):
        tk.Label(self.input_frame, text=f"{label}:").pack(anchor="w")
        ent = tk.Entry(self.input_frame)
        if default_val is not None:
            ent.insert(0, str(default_val))
        ent.pack(fill="x", pady=2)
        self.entries[label] = ent
        return ent

    def toggle_t_servo(self):
        self.t_servo_frame.pack_forget() if self.t_arm_equal.get() else self.t_servo_frame.pack(fill="x", pady=2)

    def toggle_vsafe(self):
        self.vsafe_frame.pack_forget() if self.autoset_vsafe.get() else self.vsafe_frame.pack(fill="x", pady=2)

    def submit(self):
        motion = self.motion_type.get()
        data = {k: (v.get() if hasattr(v, "get") else v) for k, v in self.entries.items()}
        if motion in ["pp_coor", "pp_angle", "pvt_coor", "pvt_angle"] and data.get("t_arm_equal", True):
            data["t_servo"] = data.get("t_arm", "")
        if motion == "pvt_csv" and data.get("Autoset_vsafe", True):
            try:
                data["v_safe"] = round(float(data.get("v_max", 0)) / 10)
            except:
                data["v_safe"] = 0
        data.pop("t_arm_equal", None)
        data.pop("Autoset_vsafe", None)
        if self.on_submit:
            self.on_submit(motion, data)
        self.destroy()


# --- GUI utama MotionDesigner ---
class MotionDesigner:
    def __init__(self, root):
        self.root = root
        self.root.title("Motion Designer")
        self.last_xyz_yaw = (0, 0, 0, 0)

        self.tree = ttk.Treeview(root, columns=("Index", "Type", "Details"), show="headings")
        for h in ["Index", "Type", "Details"]:
            self.tree.heading(h, text=h)
            self.tree.column(h, anchor="center")
        self.tree.pack(fill="both", expand=True, padx=10, pady=10)

        frame = tk.Frame(root)
        frame.pack(pady=5)
        tk.Button(frame, text="Add Motion", command=self.add_motion).pack(side="left", padx=5)
        tk.Button(frame, text="Remove Motion", command=self.remove_motion).pack(side="left", padx=5)
        tk.Button(frame, text="Save", command=self.save_motions).pack(side="left", padx=5)
        tk.Button(frame, text="Load", command=self.load_motions).pack(side="left", padx=5)

    # === Fitur pendukung ===
    def autosize_columns(self):
        import tkinter.font as tkFont
        for col in self.tree["columns"]:
            font = tkFont.Font()
            max_width = font.measure(col) + 20
            for item in self.tree.get_children():
                width = font.measure(str(self.tree.set(item, col))) + 20
                max_width = max(max_width, width)
            self.tree.column(col, width=max_width)

    # === Tambah Motion ===
    def add_motion(self):
        children = list(self.tree.get_children())
        selected = self.tree.selection()
        max_repeat_index = children.index(selected[0]) + 1 if selected else len(children)
        MotionPopup(self.root, folder=".", on_submit=self.insert_motion,
                    current_count=len(children), max_repeat_index=max_repeat_index)

    def insert_motion(self, motion_type, data):
        normalized = {k.lower(): v for k, v in data.items()}
        motion_type = motion_type.lower()

        # --- Cek CSV safety ---
        highlight = ""
        if motion_type == "pvt_csv":
            csv_name = normalized.get("csv", "")
            if csv_name and os.path.exists(csv_name):
                first = read_first_xyz_yaw(csv_name)
                if first:
                    try:
                        offset_z = float(normalized.get("offset_z", 0))
                    except:
                        offset_z = 0.0
                    dx = abs(first[0] - self.last_xyz_yaw[0])
                    dy = abs(first[1] - self.last_xyz_yaw[1])
                    dz = abs((first[2] + offset_z) - self.last_xyz_yaw[2])
                    dyaw = abs(first[3] - self.last_xyz_yaw[3])

                    out_of = []
                    if dx > 1: out_of.append(f"X (Δx={dx:.2f})")
                    if dy > 1: out_of.append(f"Y (Δy={dy:.2f})")
                    if dz > 1: out_of.append(f"Z (Δz={dz:.2f})")
                    if dyaw > 1: out_of.append(f"C (Δc={dyaw:.2f})")

                    if out_of:
                        messagebox.showwarning("Coordinate Mismatch",
                            f"⚠️ CSV '{csv_name}' start differs > ±1mm/°\n\nOut of range: {', '.join(out_of)}")
                        highlight = "#FFB3B3"

        details = ", ".join(f"{k}={v}" for k, v in normalized.items())
        children = list(self.tree.get_children())
        sel = self.tree.selection()
        new = self.tree.insert("", (children.index(sel[0]) + 1) if sel else "end",
                               values=("", motion_type, details))

        if highlight:
            self.tree.item(new, tags=("alert",))
            self.tree.tag_configure("alert", background=highlight)

        # Update last XYZ
        if motion_type in ["pp_coor", "pvt_coor"]:
            try:
                self.last_xyz_yaw = (float(normalized["x"]), float(normalized["y"]),
                                     float(normalized["z"]), float(normalized["c"]))
            except: pass
        elif motion_type == "pvt_csv":
            first = read_first_xyz_yaw(normalized.get("csv", ""))
            if first:
                x, y, z, c = first
                try: offset_z = float(normalized.get("offset_z", 0))
                except: offset_z = 0
                self.last_xyz_yaw = (x, y, z + offset_z, c)

        # Reindex
        for i, item in enumerate(self.tree.get_children(), start=1):
            vals = list(self.tree.item(item, "values")); vals[0] = i
            self.tree.item(item, values=vals)
        self.tree.selection_set(new); self.tree.focus(new); self.tree.see(new)
        self.autosize_columns()

    def remove_motion(self):
        for sel in self.tree.selection(): self.tree.delete(sel)
        for i, item in enumerate(self.tree.get_children(), start=1):
            vals = list(self.tree.item(item, "values")); vals[0] = i
            self.tree.item(item, values=vals)
        self.autosize_columns()

    # === Save / Load ===
    def save_motions(self):
        motions = [{"Index": v[0], "Type": v[1], "Details": v[2]}
                   for v in (self.tree.item(i, "values") for i in self.tree.get_children())]
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON files", "*.json")],
                                            title="Save Motions")
        if path:
            try:
                json.dump(motions, open(path, "w"), indent=2)
                messagebox.showinfo("Success", f"Saved to {path}")
            except Exception as e:
                messagebox.showerror("Error", f"Gagal save: {e}")

    def load_motions(self):
        path = filedialog.askopenfilename(defaultextension=".json",
                                          filetypes=[("JSON files", "*.json")],
                                          title="Load Motions")
        if path and os.path.exists(path):
            try:
                motions = json.load(open(path))
                for s in self.tree.get_children(): self.tree.delete(s)
                for i, m in enumerate(motions, start=1):
                    self.tree.insert("", "end", values=(i, m["Type"], m["Details"]))
                self.autosize_columns()
                messagebox.showinfo("Success", f"Loaded {len(motions)} motions")
            except Exception as e:
                messagebox.showerror("Error", f"Gagal load: {e}")


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("950x520")
    root.minsize(800, 450)
    MotionDesigner(root)
    root.mainloop()
