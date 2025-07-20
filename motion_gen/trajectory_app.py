import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
import numpy as np
import math

# ----- TRAJECTORY FROM TEST INPUT -----
raw_data = [
    ("angle", 2000, (0, 0, 0, 0)),
    ("angle", 4000, (360, 0, 0, 0)),
    ("coor", 2000, (166.82, -168, 90, 0)),
    ("coor", 2000, (107, 125, 90, 90)),
    ("coor", 2000, (107, 224, 90, 90)),
    ("coor", 2000, (107, 125, 90, 90)),
    ("coor", 2000, (107, 224, 90, 90)),
    ("coor", 2000, (107, 125, 90, 90)),
    ("coor", 2000, (107, 224, 90, 90)),
    ("coor", 2000, (107, 125, 90, 90)),
    ("coor", 2000, (107, 224, 90, 90)),
    ("coor", 2000, (107, 224, 115, 90)),
    ("coor", 2000, (107, 125, 115, 90)),
    ("coor", 2000, (166.82, -168, 115, 0)),
    ("angle", 2000, (460, 0, 0, 0)),
    ("angle", 4000, (0, 0, 0, 0))
]

# ----- KINEMATICS -----
def forward_kinematics(joints):
    j1, j2, j3, j4 = joints
    j4 *= -1.0
    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5
    theta2 = math.radians((j2 / 5) + OFFSET_2)
    theta3 = math.radians((j3 / 5) + OFFSET_3 - (j2 / 5))
    z0 = (j1 / 360) * 90
    x1 = L2 * math.cos(theta2)
    y1 = L2 * math.sin(theta2)
    x2 = x1 + L3 * math.cos(theta2 + theta3)
    y2 = y1 + L3 * math.sin(theta2 + theta3)
    yaw = (j4 / 5) + OFFSET_4
    return [x2, y2, z0, yaw]

def inverse_kinematics(tar_coor):
    x, y, z, yaw = tar_coor
    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134
    OFFSET_4 = -52.5
    distance = math.sqrt(x**2 + y**2)
    if distance > (L2 + L3):
        raise ValueError("out of boundary")
    cos_theta3 = (x**2 + y**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta3 = math.acos(cos_theta3)
    k1 = L2 + L3 * cos_theta3
    k2 = L3 * math.sin(theta3)
    theta2 = math.atan2(y, x) - math.atan2(k2, k1)
    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)
    joint_1 = z * (360 / 90)
    joint_2 = (theta2 - OFFSET_2) * 5
    joint_3 = (theta3 - OFFSET_3) * 5 + joint_2
    joint_4 = (yaw - OFFSET_4) * 5
    joint_4 *= -1
    return [joint_1, joint_2, joint_3, joint_4]

def forward_kinematics_for_plot(joints):
    j1, j2, j3, j4 = joints
    j4 *= -1.0
    L2 = 137.0
    L3 = 121.0
    OFFSET_2 = -96.5
    OFFSET_3 = 134.0
    OFFSET_4 = -52.5
    theta2 = math.radians((j2 / 5) + OFFSET_2)
    theta3 = math.radians((j3 / 5) + OFFSET_3 - (j2 / 5))
    z0 = (j1 / 360) * 90
    x1 = L2 * math.cos(theta2)
    y1 = L2 * math.sin(theta2)
    x2 = x1 + L3 * math.cos(theta2 + theta3)
    y2 = y1 + L3 * math.sin(theta2 + theta3)
    yaw = (j4 / 5) + OFFSET_4
    return [(0, 0, 0), (0, 0, z0), (x1, y1, z0), (x2, y2, z0), yaw]

# ----- TRAJECTORY GENERATION using COORDINATE SPACE -----
trajectory = []
t_ms = 0
step_ms = 50
prev = raw_data[0][2]
prev_type = raw_data[0][0]
if prev_type == "angle":
    prev = forward_kinematics(prev)

for typ, duration, values in raw_data[1:]:
    try:
        if typ == "angle":
            next_coor = forward_kinematics(values)
        else:
            next_coor = values

        steps = duration // step_ms
        for i in range(steps):
            alpha = i / steps
            interp_coor = [p + alpha * (v - p) for p, v in zip(prev, next_coor)]
            try:
                interp_joints = inverse_kinematics(interp_coor)
            except ValueError:
                interp_joints = trajectory[-1][0] if trajectory else [0, 0, 0, 0]
            x, y, z, yaw = forward_kinematics(interp_joints)
            trajectory.append((interp_joints, [x, y, z, yaw], t_ms))
            t_ms += step_ms

        prev = next_coor
        prev_type = typ
    except ValueError:
        print(f"⚠️ Skipping invalid segment from {prev} to {values} (out of boundary)")

traj = trajectory

# ----- DASH APP -----
app = dash.Dash(__name__)
app.title = "Robot Arm Viewer"

app.layout = html.Div([
    html.H2("Robot Arm Trajectory Viewer with Vertical Rail"),
    html.Div(id="coor-display", style={"marginBottom": "10px", "fontSize": "18px"}),
    dcc.Graph(id="arm-graph", style={"height": "600px"}, config={"scrollZoom": True}),
    dcc.Interval(id="interval", interval=50, n_intervals=0, disabled=True),
    html.Button("Play", id="play-btn", n_clicks=0),
    html.Button("Pause", id="pause-btn", n_clicks=0),
    dcc.Slider(id="frame-slider", min=0, max=len(traj)-1, value=0, step=1,
               tooltip={"placement": "bottom", "always_visible": True}, updatemode='drag')
])

@app.callback(
    Output("interval", "disabled"),
    [Input("play-btn", "n_clicks"), Input("pause-btn", "n_clicks")]
)
def toggle_interval(play_clicks, pause_clicks):
    return pause_clicks >= play_clicks

@app.callback(
    Output("frame-slider", "value"),
    [Input("interval", "n_intervals")],
    [State("frame-slider", "value")]
)
def update_slider(n_intervals, current_value):
    if current_value + 1 >= len(traj):
        return 0
    return current_value + 1

@app.callback(
    [Output("arm-graph", "figure"),
     Output("coor-display", "children")],
    [Input("frame-slider", "value")]
)
def update_graph(frame_idx):
    joints, coord, t = traj[frame_idx]
    points, yaw = forward_kinematics_for_plot(joints)[:-1], forward_kinematics_for_plot(joints)[-1]

    x_vals, y_vals, z_vals = zip(*points)
    lines = go.Scatter3d(x=x_vals, y=y_vals, z=z_vals, mode='lines+markers',
                         line=dict(color='blue', width=5),
                         marker=dict(size=5, color='red'))

    end = np.array(points[-1])
    arrow_len = 40
    yaw_rad = math.radians(yaw)
    arrow_dir = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0])
    arrow_end = end + arrow_len * arrow_dir
    arrow = go.Scatter3d(
        x=[end[0], arrow_end[0]],
        y=[end[1], arrow_end[1]],
        z=[end[2], arrow_end[2]],
        mode="lines",
        line=dict(color="green", width=4),
        name="Yaw Direction"
    )

    fig = go.Figure(data=[lines, arrow])
    fig.update_layout(scene=dict(
        xaxis=dict(range=[-300, 300]),
        yaxis=dict(range=[-300, 300]),
        zaxis=dict(range=[0, 150]),
        aspectmode='cube'
    ))
    fig.update_layout(margin=dict(l=0, r=0, b=0, t=30))

    coor_text = f"Moving to: x={coord[0]:.1f}, y={coord[1]:.1f}, z={coord[2]:.1f}, yaw={coord[3]:.1f}"
    return fig, coor_text

if __name__ == "__main__":
    app.run(debug=True)
