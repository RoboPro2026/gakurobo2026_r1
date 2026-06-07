echo "========== R1 setup start =========="

cd "$HOME/ros2_ws"
# 最初にvenvを有効にする
source .venv/bin/activate
# その後にinstall/setup.bashを実行
source install/setup.bash

# arucoマーカをssh経由でGUIを起動に必要なおまじない
# export XDG_RUNTIME_DIR=/run/user/$(id -u)
# export WAYLAND_DISPLAY=wayland-0
# export QT_QPA_PLATFORM=wayland
# export DBUS_SESSION_BUS_ADDRESS=unix:path=$XDG_RUNTIME_DIR/bus

sudo cpupower frequency-set -g performance

IMU_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1.4.4:1.0-port0"
LIDAR1_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.2.4:1.0"
LIDAR2_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4.1:1.0"
ARUCO_SPEAR_RED_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1.2:1.0-port0"
ARUCO_SPEAR_BLUE_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1.3:1.0-port0"
ARUCO_R2_LIFT_LOWER_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1.4.3:1.0"
ARUCO_R2_LIFT_UPPER_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1.1:1.0-port0"
YDLIDAR_FM_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.2.2:1.0-port0"
YDLIDAR_FL_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.2.3:1.0-port0"
YDLIDAR_RM_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4.3:1.0-port0"
YDLIDAR_RL_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4.2:1.0-port0"

# by-id -> 実体の ttyUSBx を取得
IMU_TTY="$(basename "$(readlink -f "$IMU_DEV")")"
LIDAR1_TTY="$(basename "$(readlink -f "$LIDAR1_DEV")")"
LIDAR2_TTY="$(basename "$(readlink -f "$LIDAR2_DEV")")"
ARUCO_RED_TTY="$(basename "$(readlink -f "$ARUCO_SPEAR_RED_DEV")")"
ARUCO_BLUE_TTY="$(basename "$(readlink -f "$ARUCO_SPEAR_BLUE_DEV")")"
ARUCO_R2_LIFT_LOWER_TTY="$(basename "$(readlink -f "$ARUCO_R2_LIFT_LOWER_DEV")")"
ARUCO_R2_LIFT_UPPER_TTY="$(basename "$(readlink -f "$ARUCO_R2_LIFT_UPPER_DEV")")"
YDLIDAR_FH_TTY="$(basename "$(readlink -f "$YDLIDAR_FH_DEV")")"
YDLIDAR_FM_TTY="$(basename "$(readlink -f "$YDLIDAR_FM_DEV")")"
YDLIDAR_FL_TTY="$(basename "$(readlink -f "$YDLIDAR_FL_DEV")")"
YDLIDAR_RH_TTY="$(basename "$(readlink -f "$YDLIDAR_RH_DEV")")"
YDLIDAR_RM_TTY="$(basename "$(readlink -f "$YDLIDAR_RM_DEV")")"
YDLIDAR_RL_TTY="$(basename "$(readlink -f "$YDLIDAR_RL_DEV")")"

sudo chmod 666 "$IMU_DEV"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/$IMU_TTY/latency_timer"
sudo chmod 666 "$LIDAR1_DEV"
sudo chmod 666 "$LIDAR2_DEV"
sudo chmod 666 "$ARUCO_SPEAR_RED_DEV"
sudo chmod 666 "$ARUCO_SPEAR_BLUE_DEV"
sudo chmod 666 "$ARUCO_R2_LIFT_LOWER_DEV"
sudo chmod 666 "$ARUCO_R2_LIFT_UPPER_DEV"

sudo chmod 666 "$YDLIDAR_FM_DEV"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/$YDLIDAR_FM_TTY/latency_timer"
sudo chmod 666 "$YDLIDAR_FL_DEV"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/$YDLIDAR_FL_TTY/latency_timer"
sudo chmod 666 "$YDLIDAR_RM_DEV"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/$YDLIDAR_RM_TTY/latency_timer"
sudo chmod 666 "$YDLIDAR_RL_DEV"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/$YDLIDAR_RL_TTY/latency_timer"

echo "---------- USB setup OK ----------"

sudo ./src/gakurobo2026_common/can_setup.bash

echo "--------- USB2CAN setup OK ----------"
echo "  ROS_DOMAIN_ID      = ${ROS_DOMAIN_ID:-(未設定, デフォルト0)}"
echo "  ROS_LOCALHOST_ONLY = ${ROS_LOCALHOST_ONLY:-(未設定, デフォルト無効)}"
echo "========= R1 setup success =========="
