# Commands from the Tkinter tester UI to robot controller
# Topic: /scara_arm/tester_ui

builtin_interfaces/Time stamp

string command          # e.g. "pp_joint", "pp_coor", "wake_up", "dancing", ...
string motor            # "all" | "stepper_only" | "servo_only"
int32 time_ms

float64[4] joints        # Joint1..Joint4 (deg). Unused commands can publish zeros.
float64[4] coor          # X,Y,Z,Yaw (mm,mm,mm,deg). Unused commands can publish zeros.

string csv_path          # Used by "dancing" command (can be empty)
