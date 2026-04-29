echo "========== R1 setup start =========="

cd "$HOME/ros2_ws"
# 最初にvenvを有効にする
source .venv/bin/activate
# その後にinstall/setup.bashを実行
source install/setup.bash

# arucoマーカをssh経由でGUIを起動に必要なおまじない
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export WAYLAND_DISPLAY=wayland-0
export QT_QPA_PLATFORM=wayland
export DBUS_SESSION_BUS_ADDRESS=unix:path=$XDG_RUNTIME_DIR/bus

sudo cpupower frequency-set -g performance

IMU_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4.4:1.0-port0"
LIDAR1_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0"
LIDAR2_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1:1.0"
ARUCO_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4.1:1.0-port0"

# by-id -> 実体の ttyUSBx を取得
IMU_TTY="$(basename "$(readlink -f "$IMU_DEV")")"

sudo chmod 666 "$IMU_DEV"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/$IMU_TTY/latency_timer"

sudo chmod 666 "$LIDAR1_DEV"
sudo chmod 666 "$LIDAR2_DEV"

sudo chmod 666 "$ARUCO_DEV"

echo "---------- IMU setup OK ----------"

sudo ./src/gakurobo2026_common/can_setup.bash

echo "--------- USB2CAN setup OK ----------"
echo "========= R1 setup success =========="
