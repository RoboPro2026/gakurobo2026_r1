# gakurobo2026_r1

`gakurobo2026_r1` は、学生ロボコン 2026 R1の ROS 2 ワークスペース用リポジトリです。  
このリポジトリには、ロボット全体の状態管理、足回り制御、機構制御、起動設定、独自メッセージ定義が入っています。

初めて見る人向けに一言でまとめると、役割は次のように分かれています。

- `r1_bringup`
  - 全ノードを起動する。
- `r1_main`
  - 操縦や自動動作の流れを決める。
- `r1_control`
  - 移動制御や経路追従など、制御理論寄りの処理を行う。
- `r1_machine`
  - モータや GPIO など実機ハードウェアとの接続を行う。
- `r1_msgs`
  - このロボット専用の ROS メッセージ定義。
- `r1_util`
  - 共通で使うプログラム。

## 全体像

このリポジトリの基本的なデータの流れは次の通りです。

1. コントローラからの入力や自動動作の状態遷移を `r1_main_node` が管理します。
2. 移動に関する指令は `r1_chassis_control_node` などが受けて、足回り用の指令へ変換します。
3. 機構の位置・速度・GPIO 指令は `r1_machine_manage_node` が Sabacan 向け topic に変換します。
4. Sabacan 系ノードや各種センサノードが、実際のハードウェアと通信します。

## リポジトリ構成

### `r1_bringup`

ROS 2 の launch ファイルとパラメータファイルを管理する package です。  

主な役割:

- 実機モードとシミュレーションモードの切り替え
- 各ノードの起動順管理
- パラメータファイルの読み込み
- Sabacan、IMU、LiDAR、制御ノードのまとめ起動

主なファイル:

- [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/launch/r1_bringup.launch.py)
  - 実機用の通常起動
- [`r1_machine_config.yaml`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml)
  - R1 全体で使う主要パラメータ

### `r1_main`

ロボット全体の高レベル制御を担当する package です。  
手動操縦、自動動作、状態遷移、初期化、Sabacan reset などを行います。
主なプログラム:

- `r1_main_node`
  - PS4 入力を受ける
  - `IDLE / MANUAL / AUTO / EMERGENCY` の状態を管理する
  - 各機構へ位置・速度・GPIO 指令を publish する
  - PS ボタン押下時にロボット初期化と `/r1_machine_initialize` publish を行う

### `r1_control`

移動制御や経路追従など、比較的アルゴリズム寄りの処理を担当する package です。

主なプログラム:

- `r1_chassis_control_node`
  - シャーシの高レベル移動指令から足回り向け指令を生成します。
- `r1_dummy_odometry_node`
  - シミュレーション時にダミーの odometry / TF を出します。
- `r1_laser_filter_node`
  - LiDAR データの前処理を行います。

補足:

- 経路生成 GUI や trajectory planner 関連のコードもこの package にあります。
- 理論寄りの処理と、テスト用ノード群もここにあります。

### `r1_machine`

実機ハードウェアとの接続を担当する package です。  
モータ、エンコーダ、GPIO、足回り、オドメトリなどのハードウェアに近い層をまとめています。

主なプログラム:

- `r1_machine_manage_node`
  - `r1_msgs` の機構用 topic と Sabacan 用 topic の変換を行います。
  - 非常停止の監視や、`/r1_machine_initialize` による復帰処理も行います。
- `r1_linear_motion_node`
  - 直動機構の位置制御と原点検出
- `r1_angle_motion_node`
  - 回転機構の位置制御と原点検出
- `r1_swerve_drive_node`
  - 独立ステアの目標値計算
- `r1_mecanum_node`
  - メカナム足回り用の変換処理
- `r1_odometry_node`
  - オドメトリ計算

### `r1_msgs`

R1 専用のメッセージ型を定義する package です。

例:

- `MotorRef`
- `LinearMotion`
- `AngleMotion`
- `Mecanum`
- `SwerveDrive`
- `RobotMove`
- GPIO 関連メッセージ

### `r1_util`

複数 package から使う共通処理をまとめた package です。  
座標変換や小さな補助関数など、独立したライブラリとして使う想定のコードが入っています。

### `data`

ロボットの動作に使用する経路データや waypoint などがあります。

### `urg_node2`

Hokuyo LiDAR 用の package です。  
この repo ではサブモジュールとして含めています。R1 固有のコードではありませんが、LiDAR を使うために一緒に管理しています。

## 実際に起動される主なノード

通常の実機起動では、`r1_bringup.launch.py` から主に次のノードが立ち上がります。

- `r1_main_node`
  - ロボット全体の進行管理
- `r1_chassis_control_node`
  - 移動制御
- `r1_swerve_drive_node`
  - 独ステ計算
- `r1_odometry_node`
  - オドメトリ計算
- `r1_machine_manage_node`
  - 機構指令と Sabacan の橋渡し
- 各 `r1_linear_motion_node` / `r1_angle_motion_node`
  - 個別機構の位置制御
- Sabacan 系ノード
  - 電源、LED、モータ、GPIO 基板との通信
- `bno086_node`
  - IMU
- `joy_node`
  - コントローラからの入力

## clone 方法

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules git@github.com:RoboPro2026/gakurobo2026_r1.git
```

## submodule の更新方法

```bash
# gakurobo2026_r1 を更新
git pull origin main
# submodule を更新
git submodule update --init --recursive
```

## インストール

### apt

```bash
sudo apt install -y python3-rosdep2 ros-humble-magic-enum ros-humble-xacro ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-laser-filters
sudo apt install -y libeigen3-dev
sudo apt install -y pybind11-dev
```

### pip

```bash
pip install numpy matplotlib pyqt6
```

### venv

```bash
python -m venv .venv
source .venv/bin/activate
```

## `urg_node2` の依存解決

```bash
cd ~/ros2_ws/src/gakurobo2026_r1
rosdep update
rosdep install -i --from-paths urg_node2
```

## ROS 2 の起動

`r1_bringup.launch.py` は `use_sim` 引数で、実機モードとシミュレーションモードを切り替えられます。  
`zone` は現在 [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/launch/r1_bringup.launch.py) 内で設定しています。

### 実機モード

現在は 2 つのターミナルを使用しています。

ターミナル 1:

```bash
cd ~/ros2_ws
./src/gakurobo2026_r1/r1_setup.bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

ターミナル 2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=false
```

`use_sim:=false` はデフォルトなので、以下でも同じです。

```bash
ros2 launch r1_bringup r1_bringup.launch.py
```

### シミュレーションモード

シミュレーションモードでは、`r1_dummy_odometry_node` が `/odometry`、`/map`、`map -> odom` TF を publish します。  
このとき `amcl` や実機用のセンサ・CAN 系ノードは起動しません。

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=true
```

## 軌道生成 GUI の実行

CSV ファイルのパスは環境に合わせて指定してください。

```bash
cd ~/ros2_ws
colcon build --symlink-install r1_control
python src/gakurobo2026_r1/src/trajectory_planner_gui.py
```

## 初見の人向けの読み順

どこから見ればよいか迷ったら、次の順がおすすめです。

1. この README で package 全体の役割を把握する
2. [`r1_bringup.launch.py`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_bringup/launch/r1_bringup.launch.py) を見て、起動されるノードを確認する
3. [`r1_main_node.md`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_main/docs/r1_main_node.md) を見て、ロボット全体の流れを把握する
4. [`r1_machine_manage_node.md`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_machine/docs/r1_machine_manage_node.md) を見て、機構指令がどのように実機へつながるか確認する
5. 必要に応じて `r1_control` や各機構ノードの docs を読む

## ドキュメント

主なドキュメント:

- [`r1_main_node.md`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_main/docs/r1_main_node.md)
- [`r1_machine/README.md`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_machine/README.md)
- [`r1_control/README.md`](/home/user/ros2_ws/src/gakurobo2026_r1/r1_control/README.md)

補足:

- 一部 docs は古い可能性があるので注意してください。

## 外部共通ライブラリについて

`bno086` や `sabacan` など、複数機体で共有しているライブラリは `gakurobo2026_common` 側にあります。  
