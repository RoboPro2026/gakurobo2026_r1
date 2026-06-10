cd "$HOME/ros2_ws"
./src/gakurobo2026_r1/scripts/r1_setup.bash
# 最初にvenvを有効にする
source .venv/bin/activate
# その後にinstall/setup.bashを実行
source install/setup.bash

# arucoマーカをssh経由でGUIを起動に必要なおまじない
# export XDG_RUNTIME_DIR=/run/user/$(id -u)
# export WAYLAND_DISPLAY=wayland-0
# export QT_QPA_PLATFORM=wayland
# export DBUS_SESSION_BUS_ADDRESS=unix:path=$XDG_RUNTIME_DIR/bus
# 使い方: ./r1_auto.bash [zone] [use_phone]
#   例: ./r1_auto.bash blue true   → blueゾーン・iPhoneコントローラ
#       ./r1_auto.bash red         → redゾーン・PS4コントローラ
#       ./r1_auto.bash             → blueゾーン・PS4コントローラ
ZONE="${1:-blue}"
USE_PHONE="${2:-false}"
export RCUTILS_COLORIZED_OUTPUT=1

LOG_DIR="$HOME/ros2_ws/log_txt_dir"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/r1_auto_$(date +%Y%m%d_%H%M%S).log"

echo "========== R1 bringup start (zone: $ZONE, use_phone: $USE_PHONE) =========="
echo "Log: $LOG_FILE"
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=false use_lidar:=true robot_control_mode:=auto use_aruco_display:=true zone:=$ZONE use_phone:=$USE_PHONE 2>&1 | tee "$LOG_FILE"