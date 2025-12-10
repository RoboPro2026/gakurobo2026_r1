# gakurobo2026_r1

TODO: setup手順ちゃんと書く

インストール
```
sudo apt install ros-humble-magic-enum
ros-humble-ament-cmake-gtest
# for eigen
sudo apt install libeigen3-dev
# for pybind11
sudo apt install pybind11-dev
```

実行
```
cd ~/ros2_ws
source install/setup.bash
sudo ./src/gakurobo2026_common/can_setup.bash
ros2 launch r1_bringup r1_bringup.launch.py
```