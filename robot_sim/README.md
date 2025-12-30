# robot_sim

Simulation/visualization launch files for the SCARA arm.

This package launches RViz2 + robot_state_publisher, and optionally a joint-state source (GUI sliders) plus a **JointState mapper**.

## What the mapper does

`joint_state_mapper` subscribes to an input JointState topic (default: `/joint_states_raw`) and republishes to an output topic (default: `/joint_states`) with this mapping:

- `joint_1_out = joint_1_in`
- `joint_2_out = joint_2_in`
- `joint_3_out = -joint_2_in + joint_3_in`
- `joint_4_out = -joint_3_in + joint_4_in`

So if you increase `joint_2`, then `joint_3_out` will decrease (because it is `joint_3_in - joint_2_in`). And if you increase `joint_3`, then `joint_4_out` will decrease (because it is `joint_4_in - joint_3_in`).

## Topics

- Input (raw): `/joint_states_raw`  
  Published by `joint_state_publisher_gui` (or by an external controller if you disable the GUI).
- Output (mapped): `/joint_states`  
  Published by `joint_state_mapper`. This is what `robot_state_publisher` consumes.

## Run (with GUI sliders)

Install GUI (optional but recommended if you want sliders):

```bash
sudo apt update
sudo apt install ros-jazzy-joint-state-publisher-gui
```

Build + launch:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_sim
source install/setup.bash
ros2 launch robot_sim sim.launch.py
```

### RViz settings persistence

By default, `robot_sim` opens RViz using a per-user config file:

- `~/.ros/rviz/robot_sim.rviz`

So if you do **File → Save Config** inside RViz, your layout/settings persist even after `colcon build`.

If you want to use your own config file location, pass it explicitly:

```bash
ros2 launch robot_sim sim.launch.py rviz_config:=/path/to/my_robot_sim.rviz
```

## External control (without GUI)

If you want to control the robot from another node/program, do **not** run `joint_state_publisher(_gui)` at the same time (otherwise you will have two publishers fighting on the same topic).

Launch RViz + robot_state_publisher + mapper, but disable the built-in joint-state publisher:

```bash
ros2 launch robot_sim sim.launch.py use_joint_state_publisher:=false
```

Now publish your own JointState messages to `/joint_states_raw`.

### Example: publish from terminal

This example publishes at 30 Hz. Use URDF joint names: `joint_1`, `joint_2`, `joint_3`, `joint_4`.

```bash
ros2 topic pub -r 30 /joint_states_raw sensor_msgs/msg/JointState "{name: ['joint_1','joint_2','joint_3','joint_4'], position: [0.0, 0.5, 0.2, 0.0]}"
```

The mapper will convert it and publish `/joint_states` where `joint_3` becomes `0.2 - 0.5 = -0.3`.

## Notes

- RViz fixed frame is stabilized by a static TF: `world -> Fixed_Vertical_Rail`.
- If the robot does not move in RViz, check:

```bash
ros2 topic echo /joint_states_raw --once
ros2 topic echo /joint_states --once
ros2 topic echo /tf --once
```

- If the robot "flickers" / snaps back to 0 sometimes, it usually means **multiple publishers** are fighting on JointState topics, or some publisher is sending incomplete JointState messages (e.g., `name` set but `position` missing).

Quick checks:

```bash
ros2 topic info /joint_states_raw --verbose
ros2 topic info /joint_states --verbose
```

You generally want:

- `/joint_states_raw`: exactly 1 publisher (GUI sliders or your external controller)
- `/joint_states`: exactly 1 publisher (`joint_state_mapper`)

### Stopping nodes (WSL)

If you close a terminal window/tab without pressing Ctrl+C, ROS 2 processes can remain running in the background ("orphaned"). That often causes topic conflicts.

Quick cleanup options:

```bash
# Stop orphaned joint_state_publisher processes
pkill -f joint_state_publisher

# Or stop all ROS 2 launch processes (more aggressive)
pkill -f "ros2 launch"
```

If you want to fully stop all processes in the WSL distro (Windows side):

```powershell
wsl --shutdown
```
