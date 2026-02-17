# gakurobo2026_r1

## clone方法
```
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/RoboPro2026/gakurobo2026_r1.git
```

## submoduleの更新方法
```
# gakurobo2026_r1を更新
git pull origin main
# submoduleを更新
git submodule update --init --recursive
```

## インストール(apt)
```
sudo apt install ros-humble-magic-enum
ros-humble-ament-cmake-gtest
# for eigen
sudo apt install libeigen3-dev
# for pybind11
sudo apt install pybind11-dev
```

## インストール(pip)
```
pip install numpy matplotlib pyqt6
```

venv作成
```
python -m venv .venv
```
venvを有効にする
```
source .venv/bin/activate
```

# urg_node
```
rosdep update
rosdep install -i --from-paths urg_node2
```

## ROS 2を実行
```
cd ~/ros2_ws
source install/setup.bash
sudo ./src/gakurobo2026_common/can_setup.bash
ros2 launch r1_bringup r1_bringup.launch.py
```

## 軌道生成GUI
CSVファイルのパスはいい感じに通してください
```
cd ~/ros2_ws
colcon build --symlink-install r1_control
python src/gakurobo2026_r1/src/trajectory_planner_gui.py
```