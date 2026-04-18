# r1_aruco_display_node

`r1_aruco_display_node` は、PC 画面上に ArUco マーカ PNG を表示する `PyQt6` ベースの ROS 2 ノードです。  
`std_msgs/msg/Int32` を購読し、受け取った整数値を `marker_id` として扱って表示を切り替えます。

GUI ライブラリは `gakurobo2026_r1` 内の既存 GUI に合わせて `PyQt6` を使用しています。  
表示画像は `data/aruco_marker/marker_<marker_id>.png` を使用します。

## トピック

- Subscribe
  - `/aruco_marker_id` (`std_msgs/msg/Int32`)
    - 受信した `data` を表示する ArUco の `marker_id` として使います。

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `topic_name` | string | `"aruco_marker_id"` | `marker_id` を購読するトピック名です。 |
| `window_title` | string | `"R1 ArUco Display"` | ウィンドウタイトルです。 |
| `initial_marker_id` | integer | `0` | 起動直後に表示するマーカ ID です。 |
| `marker_image_dir` | string | `share/r1_ui/aruco_marker` | `marker_<marker_id>.png` を探すディレクトリです。 |
| `fullscreen` | bool | `false` | `true` のとき全画面表示します。 |
| `spin_rate_hz` | double | `100.0` | ROS コールバック処理のために `rclpy.spin_once()` を回す周期 [Hz] です。 |

## 動作概要

1. 起動時に `initial_marker_id` から `marker_<marker_id>.png` を解決します。
2. `PyQt6` のウィンドウ上に PNG 画像を表示します。
3. `topic_name` で指定したトピックを購読します。
4. `Int32.data` を受け取るたびに、その値を新しい `marker_id` として `marker_<marker_id>.png` を読み込み直して表示を更新します。
5. 対応する PNG が存在しない場合は、表示は更新せず warning を出します。

## 依存

Python 依存は [`../../requirements.txt`](../../requirements.txt) に定義しています。

- `PyQt6`

未導入なら、ワークスペースの Python 環境に先に入れてください。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/.venv/bin/activate
python -m pip install -r ~/ros2_ws/src/gakurobo2026_r1/requirements.txt
```

`r1_aruco_display_node` は Python スクリプトとして install しています。  
`.venv` の `PyQt6` を使う場合は、実行時に `.venv` を有効化してください。

## ビルド

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/.venv/bin/activate
cd ~/ros2_ws
colcon build --packages-select r1_ui
source install/setup.bash
```

## 起動例

通常起動:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_ui r1_aruco_display_node
```

全画面表示:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_ui r1_aruco_display_node --ros-args -p fullscreen:=true
```

初期 ID や画像ディレクトリを変える例:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_ui r1_aruco_display_node --ros-args \
  -p initial_marker_id:=12 \
  -p marker_image_dir:=/home/user/ros2_ws/src/gakurobo2026_r1/data/aruco_marker \
  -p spin_rate_hz:=50.0
```

## 表示切替例

一度だけ切り替える:

```bash
ros2 topic pub --once /aruco_marker_id std_msgs/msg/Int32 "{data: 12}"
```

1 Hz で切り替える:

```bash
ros2 topic pub /aruco_marker_id std_msgs/msg/Int32 "{data: 3}" -r 1
```

## 注意点

- 対応する `marker_<marker_id>.png` がない場合、表示は更新されません。
- このノードは `std_msgs/msg/Int32` を入力としているため、将来「画像種類」と「マーカ ID」を別管理したくなったら、専用 message に拡張したほうが扱いやすくなります。
- 実行環境に `PyQt6` がないと GUI は起動できません。
- `.venv` に依存を追加したあとで `ros2 run r1_ui r1_aruco_display_node` が失敗する場合は、`source ~/ros2_ws/.venv/bin/activate` を行ってから再実行してください。
- `spin_rate_hz` は ROS イベント処理の周期であり、GUI の再描画 FPS を固定するものではありません。描画は起動時、トピック更新時、リサイズ時に行われます。
