cd "$HOME/ros2_ws"
source install/setup.bash

# enable_auto_select=true（自動判別）
ros2 topic pub --once /r1_init_parameter r1_msgs/msg/R1InitParameter \
  "{zone: 'red', \
  r1_kfs_value: [12, 11, -1], \
  r2_kfs_value: [-1, -1, -1, -1], \
  r2_fake_kfs_value: [-1], \
  enable_auto_select: true, \
  enable_kfs_auto_chassis: false \
}"

# enable_auto_select=false（手動指定）
# ros2 topic pub --once /r1_init_parameter r1_msgs/msg/R1InitParameter \
#  "{zone: 'blue', \
#  r1_kfs_value: [3, 4, 7], \
#  r2_kfs_value: [-1, -1, -1, -1], \
#  r2_fake_kfs_value: [-1], \
#  enable_auto_select: false, \
# enable_kfs_auto_chassis: true \
#}"

# 森1（rear_kfs）と森2（front_kfs）を指定
#ros2 topic pub --once /r1_collect_kfs r1_msgs/msg/R1CollectKfs \
#  "{forest_order: [4, 7], kfs_mechanism_type: ['rear_kfs', 'front_kfs']}"
