echo "========== R1 setup start =========="
cd $HOME/ros2_ws
source install/setup.bash
sudo cpupower frequency-set -g performance
# TODO: 他のデバイス名でも対応できるようにする
sudo chmod 666 /dev/ttyUSB0
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo "---------- IMU setup OK ----------"
sudo ./src/gakurobo2026_common/can_setup.bash
echo "--------- USB2CAN setup OK ----------"
echo "========= R1 setup success =========="
# ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
