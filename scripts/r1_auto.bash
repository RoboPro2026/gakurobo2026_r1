cd "$HOME/ros2_ws"
./src/gakurobo2026_r1/scripts/r1_setup.bash
# 最初にvenvを有効にする
source .venv/bin/activate
# その後にinstall/setup.bashを実行
source install/setup.bash
echo "========== R1 bringup start =========="
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=false use_lidar:=true robot_control_mode:=auto