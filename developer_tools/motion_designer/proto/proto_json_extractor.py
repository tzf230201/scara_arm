import json
import os

def parse_details(details_str):
    """Parse 'a=1, b=2' jadi dict (tahan spasi)."""
    parts = [p.strip() for p in details_str.split(",") if p.strip()]
    out = {}
    for p in parts:
        if "=" in p:
            k, v = p.split("=", 1)
            out[k.strip()] = v.strip()
    return out

def dict_to_details(d, order=None):
    """Convert dict ke string, jaga urutan asli kalau ada."""
    items = []
    if order:
        for k in order:
            if k in d:
                items.append(f"{k}={d[k]}")
        for k in d:
            if k not in order:
                items.append(f"{k}={d[k]}")
    else:
        items = [f"{k}={v}" for k, v in d.items()]
    return ", ".join(items)

def normalize_timings(data):
    """
    Normalisasi semua motion:
    jika t_arm != t_servo → pecah jadi 2 motion.
    Extra motion memberi SELISIH ke channel yang LEBIH PENDEK.
    """
    normalized = []
    for item in data:
        t = item.get("Type", "")
        if t.startswith("pvt") or t.startswith("pp"):
            details = parse_details(item["Details"])
            order = list(details.keys())  # simpan urutan asli

            if "t_arm" in details and "t_servo" in details:
                try:
                    t_arm = int(float(details["t_arm"]))
                    t_servo = int(float(details["t_servo"]))
                except ValueError:
                    normalized.append(item.copy())
                    continue

                # motion asli tetap masuk
                normalized.append(item.copy())

                if t_arm != t_servo:
                    diff_item = item.copy()
                    diff_details = details.copy()

                    if t_arm < t_servo:
                        # selisih ke t_arm
                        diff_details["t_arm"] = str(t_servo - t_arm)
                        diff_details["t_servo"] = "0"
                    else:
                        # selisih ke t_servo
                        diff_details["t_arm"] = "0"
                        diff_details["t_servo"] = str(t_arm - t_servo)

                    diff_item["Details"] = dict_to_details(diff_details, order)
                    normalized.append(diff_item)
                continue
        # selain motion (pause/repeat/dll) langsung masuk
        normalized.append(item.copy())
    return normalized

def expand_repeats(data, from_idx=None, to_idx=None):
    """Expand repeat (nested juga)."""
    if from_idx is not None and to_idx is not None:
        slice_data = [d for d in data if from_idx <= int(d["Index"]) <= to_idx]
    else:
        slice_data = data

    expanded = []
    for item in slice_data:
        if item["Type"] != "repeat":
            expanded.append(item.copy())
        else:
            details = parse_details(item["Details"])
            f = int(details["from_index"])
            t = int(details["to_index"])
            times = int(details["how_many_times"])
            sub_expanded = expand_repeats(data, f, t)
            for _ in range(times):
                expanded.extend([se.copy() for se in sub_expanded])
    return expanded

def reindex(data):
    """Assign Index baru berurutan mulai dari 1."""
    out = []
    for i, item in enumerate(data, start=1):
        new_item = item.copy()
        new_item["Index"] = str(i)
        out.append(new_item)
    return out

def process_motion_file(input_path, output_path="resampled/motion_list.json"):
    # 1. Load JSON
    with open(input_path, "r") as f:
        data = json.load(f)

    # 2. Normalisasi timings dulu
    normalized = normalize_timings(data)

    # 3. Expand repeat (nested)
    expanded_all = expand_repeats(normalized)

    # 4. Reindex
    final = reindex(expanded_all)

    # 5. Export
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(final, f, indent=2)

    print(f"Exported expanded motion list → {output_path}")


# ==== contoh pakai ====
if __name__ == "__main__":
    process_motion_file("motions.json")
