#!/bin/bash
sudo modprobe can_dev
sudo modprobe can_raw
sudo ip link set can0 down  # Ensure changes are applied cleanly
sudo ip link set can1 down  # Ensure changes are applied cleanly

# sudo ip link set can0 type can bitrate 1000000
# sudo ip link set can1 type can bitrate 1000000
# sudo ip link set can0 txqueuelen 65536  # if less then no permission error comes
# sudo ip link set can1 txqueuelen 65536  # if less then no permission error comes
sudo ip link set can0 type can bitrate 500000
sudo ip link set can1 type can bitrate 500000
sudo ip link set can0 txqueuelen 1000  # if less then no permission error comes
sudo ip link set can1 txqueuelen 1000  # if less then no permission error comes
sudo ip link set can0 up
sudo ip link set can1 up
