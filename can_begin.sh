sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000  # if less then no permission error comes
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set can0 up
#sudo ip link set can1 down
#sudo ip link set can1 txqueuelen 1000  # if less then no permission error comes
#sudo ip link set can1 type can bitrate 1000000 loopback off
#sudo ip link set can1 up