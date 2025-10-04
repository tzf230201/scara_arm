import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os, json

# === Flag Global ===
PP_MOTION_ENABLE = False  # kalau False, PP_Coor & PP_Angle tidak muncul


class MotionPopup(tk.Toplevel):
    def __init__(self, master, folder=".", on_submit=None, current_count=0, max_repeat_index=None):
        super().__init__(master)
        self.title("Add Motion")
        self.geometry("420x650")

        self.on_submit = on_submit
        self.folder = folder
        self.current_count = current_count
        self.max_repeat_index = max_repeat_index if max_repeat_index is not None else current_count

        # === Pilih tipe motion ===
        if PP_MOTION_ENABLE:
            default_type = "pp_coor"
        else:
            default_type = "pvt_coor"

        self.motion_type = tk.StringVar(value=default_type)

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

        # Frame utk input dinamis
        self.input_frame = tk.Frame(self)
        self.input_frame.pack(fill="both", expand=True, pady=10)

        self.update_inputs()

        # Tombol OK & Cancel
        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=5)

        ok_btn = tk.Button(btn_frame, text="OK", command=self.submit)
        ok_btn.pack(side="left", padx=5)

        cancel_btn = tk.Button(btn_frame, text="Cancel", command=self.destroy)
        cancel_btn.pack(side="left", padx=5)

        # Shortcut
        self.bind("<Escape>", lambda e: self.destroy())
        self.bind("<Return>", lambda e: self.submit())

        # Fokus default ke OK
        ok_btn.focus_set()

    def update_inputs(self):
        for w in self.input_frame.winfo_children():
            w.destroy()
        self.entries = {}
        motion = self.motion_type.get()

        # --- PP_Coor & PVT_Coor ---
        if motion in ["pp_coor", "pvt_coor"]:
            defaults = {"z": 90, "x": 258, "y": 0, "c": 0,
                        "t_arm": 4000, "t_servo": 4000}
            for label in ["z", "x", "y", "c", "t_arm"]:
                self.add_entry(label, defaults[label])

            # t_servo (hidden by default)
            self.t_servo_frame = tk.Frame(self.input_frame)
            tk.Label(self.t_servo_frame, text="t_servo:").pack(anchor="w")
            self.t_servo_entry = tk.Entry(self.t_servo_frame)
            self.t_servo_entry.insert(0, str(defaults["t_servo"]))
            self.t_servo_entry.pack(fill="x", pady=2)
            self.entries["t_servo"] = self.t_servo_entry
            self.t_servo_frame.pack_forget()

            self.t_arm_equal = tk.BooleanVar(value=True)
            chk = tk.Checkbutton(
                self.input_frame, text="t_arm = t_servo",
                variable=self.t_arm_equal, command=self.toggle_t_servo
            )
            chk.pack(anchor="w", pady=5)
            self.entries["t_arm_equal"] = self.t_arm_equal

        # --- PP_Angle & PVT_Angle ---
        elif motion in ["pp_angle", "pvt_angle"]:
            defaults = {
                "angle_1": 360, "angle_2": 0, "angle_3": 0, "angle_4": 0,
                "t_arm": 4000, "t_servo": 4000
            }
            for label in ["angle_1", "angle_2", "angle_3", "angle_4", "t_arm"]:
                self.add_entry(label, defaults[label])

            # t_servo (hidden by default)
            self.t_servo_frame = tk.Frame(self.input_frame)
            tk.Label(self.t_servo_frame, text="t_servo:").pack(anchor="w")
            self.t_servo_entry = tk.Entry(self.t_servo_frame)
            self.t_servo_entry.insert(0, str(defaults["t_servo"]))
            self.t_servo_entry.pack(fill="x", pady=2)
            self.entries["t_servo"] = self.t_servo_entry
            self.t_servo_frame.pack_forget()

            self.t_arm_equal = tk.BooleanVar(value=True)
            chk = tk.Checkbutton(
                self.input_frame, text="t_arm = t_servo",
                variable=self.t_arm_equal, command=self.toggle_t_servo
            )
            chk.pack(anchor="w", pady=5)
            self.entries["t_arm_equal"] = self.t_arm_equal

        # --- PVT_CSV ---
        elif motion == "pvt_csv":
            tk.Label(self.input_frame, text="CSV File:").pack(anchor="w")
            files = [f for f in os.listdir(self.folder) if f.endswith(".csv")]
            self.entries["csv"] = ttk.Combobox(
                self.input_frame, values=files, state="readonly"
            )
            self.entries["csv"].pack(fill="x", pady=2)
            if files:
                self.entries["csv"].current(0)

            v_max_default = 100
            acc_default = 2000
            v_safe_default = v_max_default / 10

            self.add_entry("v_max", v_max_default)
            self.add_entry("acc_dec", acc_default)

            self.vsafe_frame = tk.Frame(self.input_frame)
            tk.Label(self.vsafe_frame, text="v_safe:").pack(anchor="w")
            self.vsafe_entry = tk.Entry(self.vsafe_frame)
            self.vsafe_entry.insert(0, str(v_safe_default))
            self.vsafe_entry.pack(fill="x", pady=2)
            self.entries["v_safe"] = self.vsafe_entry
            self.vsafe_frame.pack_forget()

            self.autoset_vsafe = tk.BooleanVar(value=True)
            chk = tk.Checkbutton(
                self.input_frame, text="Autoset v_safe by system",
                variable=self.autoset_vsafe, command=self.toggle_vsafe
            )
            chk.pack(anchor="w", pady=5)
            self.entries["Autoset_vsafe"] = self.autoset_vsafe

        # --- Pause ---
        elif motion == "pause":
            self.add_entry("t_wait", 1000)

        # --- Repeat ---
        elif motion == "repeat":
            if self.current_count < 1:
                tk.Label(
                    self.input_frame,
                    text="list is empty (minimum required 1 motion to repeat)"
                ).pack(anchor="w")
                return

            max_idx = max(1, self.max_repeat_index)

            tk.Label(self.input_frame, text="from_index:").pack(anchor="w")
            from_cb = ttk.Combobox(
                self.input_frame,
                values=list(range(1, max_idx + 1)),
                state="readonly"
            )
            from_cb.set(1)
            from_cb.pack(fill="x", pady=2)
            self.entries["from_index"] = from_cb

            tk.Label(self.input_frame, text="to_index:").pack(anchor="w")
            to_cb = ttk.Combobox(
                self.input_frame,
                values=list(range(1, max_idx + 1)),
                state="readonly"
            )
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
        tk.Label(self.input_frame, text=label + ":").pack(anchor="w")
        ent = tk.Entry(self.input_frame)
        if default_val is not None:
            ent.insert(0, str(default_val))
        ent.pack(fill="x", pady=2)
        self.entries[label] = ent
        return ent

    def toggle_t_servo(self):
        if self.t_arm_equal.get():
            self.t_servo_frame.pack_forget()
        else:
            self.t_servo_frame.pack(fill="x", pady=2)

    def toggle_vsafe(self):
        if self.autoset_vsafe.get():
            self.vsafe_frame.pack_forget()
        else:
            self.vsafe_frame.pack(fill="x", pady=2)

    def submit(self):
        motion = self.motion_type.get()
        data = {}
        for k, v in self.entries.items():
            if isinstance(v, tk.BooleanVar):
                data[k] = v.get()
            elif hasattr(v, "get"):
                data[k] = v.get()
            else:
                data[k] = v

        if motion in ["pp_coor", "pp_angle", "pvt_coor", "pvt_angle"]:
            if data.get("t_arm_equal", True):
                data["t_servo"] = data.get("t_arm", "")

        if motion == "pvt_csv":
            try:
                v_max_val = float(data.get("v_max", 0))
            except:
                v_max_val = 0
            if data.get("Autoset_vsafe", True):
                data["v_safe"] = round(v_max_val / 10)

        data.pop("t_arm_equal", None)
        data.pop("Autoset_vsafe", None)

        if motion == "repeat" and self.current_count < 1:
            self.destroy()
            return

        if self.on_submit:
            self.on_submit(motion, data)
        self.destroy()


class MotionDesigner:
    def __init__(self, root):
        self.root = root
        self.root.title("Motion Designer")

        self.tree = ttk.Treeview(
            root, columns=("Index", "Type", "Details"), show="headings"
        )
        self.tree.heading("Index", text="Index")
        self.tree.heading("Type", text="Motion Type")
        self.tree.heading("Details", text="Details")
        self.tree.column("Index", width=50, anchor="center")
        self.tree.pack(fill="both", expand=True, padx=10, pady=10)

        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=5)

        tk.Button(btn_frame, text="Add Motion",
                  command=self.add_motion).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Remove Motion",
                  command=self.remove_motion).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Save",
                  command=self.save_motions).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Load",
                  command=self.load_motions).pack(side="left", padx=5)

    def add_motion(self):
        children = list(self.tree.get_children())
        selected = self.tree.selection()
        if selected:
            sel_index_1based = children.index(selected[0]) + 1
            max_repeat_index = sel_index_1based
        else:
            max_repeat_index = len(children)

        MotionPopup(
            self.root,
            folder=".",
            on_submit=self.insert_motion,
            current_count=len(children),
            max_repeat_index=max_repeat_index
        )

    def insert_motion(self, motion_type, data):
        filtered_data = {k: v for k, v in data.items()
                         if k not in ("t_arm_equal", "Autoset_vsafe")}
        normalized = {k.lower(): v for k, v in filtered_data.items()}
        motion_type = motion_type.lower()

        details = ", ".join(f"{k}={v}" for k, v in normalized.items())

        children = list(self.tree.get_children())
        selected = self.tree.selection()

        if selected:
            sel_index = children.index(selected[0])
            new_item = self.tree.insert("", sel_index + 1, values=("", motion_type, details))
        else:
            new_item = self.tree.insert("", "end", values=("", motion_type, details))

        for i, item in enumerate(self.tree.get_children(), start=1):
            vals = list(self.tree.item(item, "values"))
            vals[0] = i
            self.tree.item(item, values=vals)

        self.tree.selection_set(new_item)
        self.tree.focus(new_item)
        self.tree.see(new_item)

    def remove_motion(self):
        selected = self.tree.selection()
        for sel in selected:
            self.tree.delete(sel)
        for i, item in enumerate(self.tree.get_children(), start=1):
            vals = list(self.tree.item(item, "values"))
            vals[0] = i
            self.tree.item(item, values=vals)

    def save_motions(self):
        motions = []
        for item in self.tree.get_children():
            idx, mtype, details = self.tree.item(item, "values")
            motions.append({"Index": idx, "Type": mtype, "Details": details})

        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
            title="Save Motions"
        )
        if file_path:
            try:
                with open(file_path, "w") as f:
                    json.dump(motions, f, indent=2)
                messagebox.showinfo("Success", f"Motions saved to {file_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Gagal save: {e}")

    def load_motions(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
            title="Load Motions"
        )
        if file_path and os.path.exists(file_path):
            try:
                with open(file_path, "r") as f:
                    motions = json.load(f)
                for sel in self.tree.get_children():
                    self.tree.delete(sel)
                for i, m in enumerate(motions, start=1):
                    self.tree.insert("", "end", values=(i, m["Type"], m["Details"]))
                if motions:
                    last_item = self.tree.get_children()[-1]
                    self.tree.selection_set(last_item)
                    self.tree.focus(last_item)
                    self.tree.see(last_item)
                messagebox.showinfo("Success", f"Loaded {len(motions)} motions")
            except Exception as e:
                messagebox.showerror("Error", f"Gagal load: {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = MotionDesigner(root)
    root.mainloop()
