from uim342ab import *

def print_yellow(text):
    YELLOW = "\033[93m"
    RESET = "\033[0m"
    print(f"{YELLOW}{text}{RESET}")

node = 6

bitrate = uim342ab_get_bitrate(node)
print_yellow(f"Bitrate saat ini: {bitrate}")

node_id = uim342ab_get_node_id(node)
print_yellow(f"Node ID saat ini: {node_id}")

group_id = uim342ab_get_group_id(node)
print_yellow(f"Group ID saat ini: {group_id}")

ac_dc = uim342ab_get_ac_dc_unit(node)
unit = 'milliseconds' if ac_dc == 1 else 'pulses' if ac_dc == 0 else 'Unknown'
print_yellow(f"AC/DC unit: {ac_dc} ({unit})")

loop = uim342ab_get_using_close_loop(node)
loop_mode = 'Closed-loop' if loop == 1 else 'Open-loop' if loop == 0 else 'Unknown'
print_yellow(f"Closed-loop mode: {loop} ({loop_mode})")
