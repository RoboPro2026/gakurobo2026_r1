cd "$HOME/ros2_ws"
./src/gakurobo2026_r1/scripts/r1_setup.bash
# 最初にvenvを有効にする
source .venv/bin/activate
# その後にinstall/setup.bashを実行
source install/setup.bash

# arucoマーカをssh経由でGUIを起動に必要なおまじない
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export WAYLAND_DISPLAY=wayland-0
export QT_QPA_PLATFORM=wayland
export DBUS_SESSION_BUS_ADDRESS=unix:path=$XDG_RUNTIME_DIR/bus
echo "========== R1 bringup start =========="
USE_PHONE=${1:-false}
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=false use_lidar:=true use_phone:=$USE_PHONE