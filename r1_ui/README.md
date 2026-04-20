# r1_ui

`r1_ui` は `gakurobo2026_r1` の UI 系ノードを置く ROS 2 パッケージです。

現在は以下のノードを含みます。

- `r1_aruco_display_node`
  - PC 画面上に ArUco マーカを表示する `PyQt6` ベースのノードです。
  - `std_msgs/msg/Int32` を購読し、受け取った `marker_id` に対応する PNG を `data/aruco_marker` から選んで表示します。
- `r1_aruco_serial_node`
  - シリアル通信で小型ディスプレイへ ArUco マーカ ID を送る C++ ノードです。
  - `std_msgs/msg/Int32` を購読し、受け取った `marker_id` を改行付き文字列でシリアル送信します。

詳細は以下を参照してください。

- [`docs/r1_aruco_display_node.md`](./docs/r1_aruco_display_node.md)
- [`docs/r1_aruco_serial_node.md`](./docs/r1_aruco_serial_node.md)

Python 依存は [`../requirements.txt`](../requirements.txt) にまとめています。

`r1_aruco_display_node` は Python スクリプトとして install しています。  
`.venv` の `PyQt6` を使う場合は、実行時にその環境を有効化してください。
