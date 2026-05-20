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
| `screen_name` | string | `""` | 表示先画面の名前です。空文字のときはQtのデフォルト画面を使います。指定した画面が見つからない場合はウィンドウを表示しません。非全画面表示でも指定できます。 |
| `image_rotation_degrees` | integer | `0` | 表示画像の回転角度 [deg] です。モニターを上下逆に取り付けた場合は `180` を指定します。 |
| `marker_x` | integer | `-1` | マーカ表示矩形の左上 X 座標 [px] です。`-1` のときは従来どおり中央表示します。 |
| `marker_y` | integer | `-1` | マーカ表示矩形の左上 Y 座標 [px] です。`-1` のときは従来どおり中央表示します。 |
| `marker_width` | integer | `-1` | マーカ表示矩形の幅 [px] です。`-1` のときは従来どおり中央表示します。 |
| `marker_height` | integer | `-1` | マーカ表示矩形の高さ [px] です。`-1` のときは従来どおり中央表示します。 |
| `spin_rate_hz` | double | `100.0` | ROS コールバック処理のために `rclpy.spin_once()` を回す周期 [Hz] です。 |

`marker_x` / `marker_y` / `marker_width` / `marker_height` は4つすべて指定したときだけ有効です。
座標は表示ウィンドウ左上基準です。`fullscreen:=true` のときは、全画面表示されたウィンドウ内で画像の表示位置と大きさを指定します。

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

画面名確認用の `scripts/list_screen_names.py` も `PyQt6` を使います。  
`r1_aruco_display_node` と同じ Qt の画面情報を確認するため、実行時は `.venv` を有効化してください。

## ビルド

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/.venv/bin/activate
cd ~/ros2_ws
colcon build --packages-select r1_ui
source install/setup.bash
```

## 表示環境

このノードは GUI を表示するため、実行環境に `DISPLAY` または `WAYLAND_DISPLAY` が必要です。  
ロボット PC に SSH で入っただけのシェルでは、通常これらが設定されていないため起動できません。

まず以下で表示環境を確認してください。

```bash
echo $DISPLAY
echo $WAYLAND_DISPLAY
echo $XDG_SESSION_TYPE
```

空の場合は、ログイン済みのデスクトップ端末から起動してください。  
Wayland セッションでは、`QT_QPA_PLATFORM` が未指定の場合にノード側で `wayland` backend を優先します。

X11 環境で以下のような `xcb` 関連エラーが出る場合は、OS パッケージが不足しています。

```text
Could not load the Qt platform plugin "xcb"
```

Ubuntu 22.04 では以下を入れてください。

```bash
sudo apt install libxcb-cursor0
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

画面名を確認:

```bash
source ~/ros2_ws/.venv/bin/activate
~/ros2_ws/src/gakurobo2026_r1/scripts/list_screen_names.py
```

特定画面に全画面表示:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_ui r1_aruco_display_node --ros-args \
  -p fullscreen:=true \
  -p screen_name:=HDMI-1
```

特定画面に通常ウィンドウ表示:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_ui r1_aruco_display_node --ros-args \
  -p fullscreen:=false \
  -p screen_name:=HDMI-1
```

画像を180度回転して表示:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_ui r1_aruco_display_node --ros-args \
  -p fullscreen:=true \
  -p screen_name:=HDMI-1 \
  -p image_rotation_degrees:=180
```

全画面内で画像の位置と大きさを指定して表示:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_ui r1_aruco_display_node --ros-args \
  -p fullscreen:=true \
  -p screen_name:=HDMI-1 \
  -p marker_x:=80 \
  -p marker_y:=120 \
  -p marker_width:=480 \
  -p marker_height:=480
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

# ssh経由でGUIを起動したいときは
接続先でターミナルを起動しておけば、次のおまじないを入力すれば行ける
```bash
source ~/ros2_ws/.venv/bin/activate
source ~/ros2_ws/install/setup.bash

export XDG_RUNTIME_DIR=/run/user/$(id -u)
export WAYLAND_DISPLAY=wayland-0
export QT_QPA_PLATFORM=wayland
export DBUS_SESSION_BUS_ADDRESS=unix:path=$XDG_RUNTIME_DIR/bus
```

## 注意点

- 対応する `marker_<marker_id>.png` がない場合、表示は更新されません。
- `screen_name` は `scripts/list_screen_names.py` に表示される `name=...` の値を指定してください。存在しない名前を指定した場合は warning を出し、ウィンドウを表示しません。
- `marker_x` / `marker_y` / `marker_width` / `marker_height` は全画面内でのマーカ表示位置を固定したいときに使います。4つのうち一部だけ指定した場合や、負の座標、0以下の幅・高さはエラーになります。
- SSH から直接起動する場合でも、ロボット PC 側の GUI セッションへ接続できる `DISPLAY` または `WAYLAND_DISPLAY` が必要です。
- このノードは `std_msgs/msg/Int32` を入力としているため、将来「画像種類」と「マーカ ID」を別管理したくなったら、専用 message に拡張したほうが扱いやすくなります。
- 実行環境に `PyQt6` がないと GUI は起動できません。
- `.venv` に依存を追加したあとで `ros2 run r1_ui r1_aruco_display_node` が失敗する場合は、`source ~/ros2_ws/.venv/bin/activate` を行ってから再実行してください。
- `spin_rate_hz` は ROS イベント処理の周期であり、GUI の再描画 FPS を固定するものではありません。描画は起動時、トピック更新時、リサイズ時に行われます。
