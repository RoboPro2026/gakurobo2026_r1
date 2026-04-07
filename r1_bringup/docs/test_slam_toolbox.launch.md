# test_slam_toolbox.launch.py

`slam_toolbox` の単体確認用 launch です。実験用のファイルで、現状は LiDAR lifecycle node や TF の起動部分がコメントアウトされ、`slam_toolbox` のみを起動する構成になっています。

## 主な役割

- `async_slam_toolbox_node` を起動する
- `r1_slam_config.yaml` を `slam_toolbox` の設定として読み込む

## 引数

- `auto_start`
- `node_name`
- `scan_topic_name`

上記 3 つは宣言されていますが、現状の launch 本体では未使用です。LiDAR lifecycle node の起動部を戻した場合に使う前提の名残です。

## 主に起動するノード

- `slam_toolbox`

## 補足

- `lifecycle_node`、event handler、LiDAR TF のコードは残っていますがコメントアウトされています。
- 実運用の自己位置推定系は通常 [`r1_slam.launch.py`](../launch/r1_slam.launch.py) を使います。

## 起動例

```bash
ros2 launch r1_bringup test_slam_toolbox.launch.py
```
