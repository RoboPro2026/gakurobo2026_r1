cd "$HOME/ros2_ws"
./src/gakurobo2026_r1/scripts/r1_setup.bash
echo "========== R1 bringup start =========="
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=false use_lidar:=true