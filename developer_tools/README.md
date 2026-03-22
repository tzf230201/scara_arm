# Developer Tools

This folder contains internal tools used to design motion data, inspect CSV trajectories, schedule predefined tasks, and monitor the live PVT stream sent to the robot controller.

## Folder Overview

### `motion_designer/`

Used to build motion sequences and export them into `.pvt`.

Important files:

- `motion_designer.py`  
  GUI editor for creating motion steps.
- `pvt_generator.py`  
  Converts motion JSON or CSV into PVT output.
- `motions.json`  
  Example saved motion list.
- `motions.pvt`  
  Example generated PVT file.
- `motions_pvt.csv`  
  Example generated CSV output.
- `motions_csv/`  
  Sample input CSV files.
- `resampled/`  
  Intermediate generated files.
- `proto/`  
  Prototype and experimental scripts.

### `task_gui/`

Used to scan predefined task CSV files, convert Cartesian path points into joint-space PVT points, and stream them over ZMQ.

Important files:

- `task_gui.py`  
  Main scheduler GUI.
- `config.json`  
  Runtime configuration.
- `csv_viewer.py`  
  CSV inspection tool with 3D visualization.
- `pvt_stream_viewer.py`  
  Live ZMQ PVT stream viewer.
- `csv_files/`  
  Task motion packs such as park, pickup, place, shuttle, turn, and home.

---

## 1. `motion_designer/motion_designer.py`

This GUI lets developers build a motion list using indexed steps and different motion types, then export the result through `pvt_generator.py`.

Run:

```bash
cd developer_tools/motion_designer
python3 motion_designer.py
```

Supported motion types:

- `pvt_coor`
  - fields: `z, x, y, c, t_arm, t_servo`
- `pvt_angle`
  - fields: `angle_1, angle_2, angle_3, angle_4, t_arm, t_servo`
- `pvt_csv`
  - fields: `csv, v_max, v_safe, acc_dec, offset_z`
- `pause`
  - field: `t_wait`
- `repeat`
  - fields: `from_index, to_index, how_many_times`

Behavior notes:

- `pvt_csv` can auto-insert a starting `pvt_coor` step based on the first row of the selected CSV.
- Arm time and servo time can be locked together from the GUI.
- Export is handled by calling `pvt_generator.py` from the editor.

Outputs:

- Saved motion list JSON.
- Generated `.pvt` file.
- Optional preview/animation during export.

---

## 2. `task_gui/task_gui.py`

This is the main developer GUI for testing Pickup, Place, Shuttle, Park, Turn, and Home task flows.

Main responsibilities:

- Scan task CSV files from the configured motion folder.
- Classify files by task type based on filename and prefix code.
- Convert `X/Y/Z/Yaw` style path points into SCARA joint targets.
- Generate interpolated PVT points using the configured motion profile.
- Stream each PVT point through ZMQ to the consumer side.

Run:

```bash
cd developer_tools/task_gui
python3 task_gui.py
```

### Config file

Default config path:

- `developer_tools/task_gui/config.json`

Override with environment variable:

```bash
export TASK_GUI_CONFIG=/path/to/config.json
```

Important config keys:

- `motions_dir`
  - Folder to scan for task CSV files.
  - Current default: `./csv_files`
- `z_by_task`
  - Z offset rules for `pickup`, `place`, `shuttle`, and `park`
- `pvt.dt_ms`
  - Time step per generated point
- `profile`
  - Interpolation and velocity profile settings
- `straight_pre_motion`
  - Optional pre-motion behavior before entering the main path
- `zmq`
  - `endpoint`
  - `mode`: `PUB_BIND`, `PUB_CONNECT`, or `PUSH_CONNECT`
  - `dry_run`

Outputs:

- Live ZMQ messages in this format:

```json
{"command":"pvt_point","p1":0,"v1":0,"p2":0,"v2":0,"p3":0,"v3":0,"p4":0,"v4":0,"t_ms":50}
```

- GUI runtime logs for scan, planning, and execution states.
- Updated config file when saved from the settings UI.

---

## 3. `task_gui/pvt_stream_viewer.py`

This tool listens to the PVT ZMQ stream and visualizes the reconstructed robot path.

Run:

```bash
cd developer_tools/task_gui
python3 pvt_stream_viewer.py
```

Features:

- Receives streamed `pvt_point` messages from ZMQ.
- Reconstructs the path into a live 3D view.
- Displays stream status and update rate.
- Can be used to verify whether `task_gui.py` is sending data correctly.

---

## 4. `task_gui/csv_viewer.py`

This is a standalone CSV inspection tool for checking task path files before they are used by `task_gui.py`.

Run:

```bash
cd developer_tools/task_gui
python3 csv_viewer.py
```

Expected CSV columns:

- `X`
- `Y`
- `Z`

Useful for:

- Checking path shape quickly in 3D.
- Confirming start/end position.
- Verifying that exported CSV files look correct before streaming.

---

## Task CSV Naming Notes

`task_gui.py` classifies task files using both filename keywords and prefix codes.

Current code mapping:

- `00` -> park
- `01` or `10` -> pickup
- `02` -> place
- `03` -> shuttle
- `99` -> turn
- `home` in filename -> home

Examples from the current `csv_files/` folder:

- `00-park_pickup_20251017.csv`
- `01-pickup_shelf_20251017_inup_outdown.csv`
- `02-place_shelf_20260304_inup_outdown.csv`
- `03-shuttle_pickup_20251017_inup_outdown.csv`
- `99-turn_place_20251017.csv`
- `home_20251017.csv`

Keeping this naming format consistent helps the GUI auto-detect the correct task category.
