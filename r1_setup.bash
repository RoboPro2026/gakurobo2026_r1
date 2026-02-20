cd ~/ros2_ws
source install/setup.bash
# TODO: 他のデバイス名でも対応できるようにする
sudo chmod 666 /dev/ttyUSB0
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo ./~/ros2_ws/src/gakurobo2026_common/can_setup.bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
