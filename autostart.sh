sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 dbitrate 2000000 sample-point 0.8 fd on
sudo ip link set can0 up
sudo sh -c 'echo 4096 > /sys/class/net/can0/tx_queue_len'