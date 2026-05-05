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
# ゾーンを第1引数から取得する。引数が省略された場合は "blue" をデフォルト値として使う
# 使い方: ./r1_auto.bash         → blue
#         ./r1_auto.bash red     → red
ZONE="${1:-blue}"
echo "========== R1 bringup start (zone: $ZONE) =========="
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=false use_lidar:=true robot_control_mode:=auto use_aruco_display:=true zone:=$ZONE