# r1_ui

`r1_ui` は `gakurobo2026_r1` の UI 系ノードを置く ROS 2 パッケージです。

現在は以下のノードを含みます。

- `r1_aruco_display_node`
  - PC 画面上に ArUco マーカを表示する `PyQt6` ベースのノードです。
  - `std_msgs/msg/Int32` を購読し、受け取った `marker_id` に応じて表示するマーカ画像を切り替えます。

詳細は [`docs/r1_aruco_display_node.md`](./docs/r1_aruco_display_node.md) を参照してください。

Python 依存は [`../requirements.txt`](../requirements.txt) にまとめています。

`r1_aruco_display_node` は Python スクリプトとして install しています。  
`.venv` の `PyQt6` や OpenCV を使う場合は、実行時にその環境を有効化してください。
