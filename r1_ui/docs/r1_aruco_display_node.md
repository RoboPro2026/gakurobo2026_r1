# r1_aruco_display_node

`r1_aruco_display_node` は、PC 画面上に ArUco マーカを表示する `PyQt6` ベースの ROS 2 ノードです。  
`std_msgs/msg/Int32` を購読し、受け取った整数値を `marker_id` として扱って表示を切り替えます。

GUI ライブラリは `gakurobo2026_r1` 内の既存 GUI に合わせて `PyQt6` を使用しています。  
ArUco マーカの生成には OpenCV の `cv2.aruco` を使用します。

## トピック

- Subscribe
  - `/aruco_marker_id` (`std_msgs/msg/Int32`)
    - 受信した `data` を表示する ArUco の `marker_id` として使います。

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `topic_name` | string | `"aruco_marker_id"` | `marker_id` を購読するトピック名です。 |
| `window_title` | string | `"R1 ArUco Display"` | ウィンドウタイトルです。 |
| `dictionary` | string | `"DICT_4X4_50"` | 使用する ArUco 辞書名です。OpenCV の `cv2.aruco` にある辞書名を指定します。 |
| `initial_marker_id` | integer | `0` | 起動直後に表示するマーカ ID です。 |
| `marker_size_px` | integer | `600` | 生成するマーカ画像の一辺のピクセル数です。 |
| `fullscreen` | bool | `false` | `true` のとき全画面表示します。 |
| `spin_rate_hz` | double | `100.0` | ROS コールバック処理のために `rclpy.spin_once()` を回す周期 [Hz] です。 |

## 動作概要

1. 起動時に `dictionary` と `initial_marker_id` を使って ArUco 画像を生成します。
2. `PyQt6` のウィンドウ上に生成したマーカを表示します。
3. `topic_name` で指定したトピックを購読します。
4. `Int32.data` を受け取るたびに、その値を新しい `marker_id` として画像を再生成して表示を更新します。
5. 指定した `marker_id` が辞書の範囲外なら、表示は更新せず warning を出します。

## 依存

Python 依存は [`../../requirements.txt`](../../requirements.txt) に定義しています。

- `PyQt6`
- `opencv-contrib-python`

未導入なら、ワークスペースの Python 環境に先に入れてください。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/.venv/bin/activate
python -m pip install -r ~/ros2_ws/src/gakurobo2026_r1/requirements.txt
```

`r1_aruco_display_node` は `ament_python` パッケージとして install されるため、build 時に使った Python interpreter が実行スクリプトへ反映されます。  
`.venv` の `PyQt6` や `opencv-contrib-python` を使う場合は、`.venv` を有効化した状態で build してください。

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

辞書や初期 ID を変える例:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_ui r1_aruco_display_node --ros-args \
  -p dictionary:=DICT_5X5_100 \
  -p initial_marker_id:=12 \
  -p marker_size_px:=900 \
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

- `dictionary` に対して無効な `marker_id` を送ると表示は更新されません。
- このノードは `std_msgs/msg/Int32` を入力としているため、将来「画像種類」と「マーカ ID」を別管理したくなったら、専用 message に拡張したほうが扱いやすくなります。
- 実行環境に `PyQt6` がないと GUI は起動できません。
- `.venv` に依存を追加したあとで `ros2 run r1_ui r1_aruco_display_node` が失敗する場合は、`.venv` を有効化した状態で再度 `colcon build --packages-select r1_ui` を実行してください。
- `spin_rate_hz` は ROS イベント処理の周期であり、GUI の再描画 FPS を固定するものではありません。描画は起動時、トピック更新時、リサイズ時に行われます。
