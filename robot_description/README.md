## View robot in RViz2 (ROS 2 Jazzy)

### 1) Make sure ROS 2 Jazzy is available

In a fresh terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

If `ros2` is still "command not found", ROS 2 Jazzy is not installed in this Ubuntu/WSL environment.

### 2) (Optional) Install joint state publisher GUI

If you want sliders for joints:

```bash
sudo apt update
sudo apt install ros-jazzy-joint-state-publisher-gui
```

If you do not install the GUI, the launch file will fall back to `joint_state_publisher` automatically.

### 3) Build and launch

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_description
source install/setup.bash
ros2 launch robot_description display.launch.py
```

### Troubleshooting

- If the robot/meshes do not appear in RViz, check `Displays -> RobotModel -> Status` for errors.
- Ensure `/joint_states` and `/tf` are being published:

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /tf --once
```