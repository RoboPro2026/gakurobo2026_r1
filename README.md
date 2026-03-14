# gakurobo2026_r1

## clone方法
```
cd ~/ros2_ws/src
git clone --recurse-submodules git@github.com:RoboPro2026/gakurobo2026_r1.git
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
sudo apt install -y ros-humble-magic-enum ros-humble-xacro ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-laser-filters
# for eigen
sudo apt install -y libeigen3-dev
# for pybind11
sudo apt install -y pybind11-dev
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

# ROS 2を実行
現在はプログラムの実行に2つのターミナルを使用しています。  
将来的には1つのターミナルにする予定です。  
ターミナル1  
```
cd ~/ros2_ws
./src/gakurobo2026_r1/r1_setup.bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

ターミナル2  
```
cd ~/ros2_ws
source install/setup.bash
ros2 launch r1_bringup r1_bringup.launch.py
```

# 軌道生成GUIの実行
CSVファイルのパスはいい感じに通してください
```
cd ~/ros2_ws
colcon build --symlink-install r1_control
python src/gakurobo2026_r1/src/trajectory_planner_gui.py
```

# プログラムの構成（自作プログラムのみ）
- data
  - 経路データ(waypoint)等。
- r1_bringup
  - ROS 2の起動に使用するPythonスクリプト。
  - 各種パラメータはr1_bringup/configに入っている。
- r1_control
  - ロボットの制御に関する部分。
  - r1_mainにも制御に関するプログラムは含まれているが、ここは理論寄り。（経路生成、経路追従など）
- r1_machine
  - ハードウェアの抽象化的な立ち位置。
  - r1_sabacan_msgs_converter_nodeで各種トピックの整理を行っている。（実装は汚い）
- r1_main
  - ロボット全体のシーケンスや操縦を管理するノード。
- r1_msgs
  - r1で使用する独自型が入っている。

共有で使用するライブラリ（`bno086`、`sabacan`）などはgakurobo2026_commonに入ってます。

各種プログラムのドキュメントは、docsの中に入っています。なお、docsの中に入っている資料は、古かったり、Codexに書かせているので内容がおかしかったりする可能性があるので注意。
