# r1_laser_filter_node

`r1_laser_filter_node` は、`sensor_msgs/msg/LaserScan` の隣接ビーム同士の幾何関係を見て、不要な点を `inf` に置き換えたフィルタ済みスキャンを publish する ROS 2 ノードです。入出力トピック名はパラメータで変更できます。

## フィルタの意図

このフィルタは、隣接する 2 本のビームがほぼ同じ向き、または正反対に近い向きを向いている点列を落とし、角が立った変化を持つ点を残しやすくすることを意図しています。

実装では、隣接 2 点を 2 次元ベクトルに変換し、その内積から cosine 値を計算しています。`|cos|` が大きいときは 2 ベクトルのなす角が 0 度または 180 度に近く、`|cos|` が小さいときは 90 度に近い関係です。このノードでは、90 度に近い変化を比較的「良い点」とみなし、0 度または 180 度に近い組を除外します。

そのため、壁面のように滑らかに並ぶ点列や、見え方として一直線に近い並びを抑え、エッジに近い反応を残したい場合に向いています。一方で、一般的な障害物除去やノイズ除去を目的とした汎用フィルタではありません。

## トピック

デフォルト設定は以下です。

- Subscribe
  - `/scan` (`sensor_msgs/msg/LaserScan`): 入力の LiDAR スキャン
- Publish
  - `/filtered_scan` (`sensor_msgs/msg/LaserScan`): フィルタ後のスキャン。除外した要素は `inf` に置き換えられます。

## 主なパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `threshold` | double | `0.8` | 隣接 2 点から計算した cosine 値の絶対値 `|cos|` に対するしきい値。`|cos| >= threshold` のとき、先頭側のビームを無効化します。 |
| `scan_topic` | string | `"/scan"` | 購読する入力 `LaserScan` トピック名。 |
| `filtered_scan_topic` | string | `"/filtered_scan"` | publish する出力 `LaserScan` トピック名。 |

## 動作概要

1. `/scan` を受け取るたびに、隣接する `ranges[i]` と `ranges[i+1]` を順に見ます。
2. どちらかが `NaN` や `inf` の場合は、その組をスキップします。
3. 各レンジを極座標から 2 次元ベクトル `(x, y)` に変換し、内積から 2 本のビーム間の cosine 値を計算します。
4. `abs(cosine)` が `threshold` 以上なら、`ranges[i]` を `inf` に置き換えます。
5. 加工後の `LaserScan` を `/filtered_scan` に publish します。

この実装は隣接点の形状変化だけを見て単純に除外するもので、クラスタリング、壁面推定、外れ値推定は行いません。

## 起動例

```bash
source ~/ros2_ws/install/setup.bash
ros2 run r1_control r1_laser_filter_node --ros-args \
  -p threshold:=0.8 \
  -p scan_topic:=/scan \
  -p filtered_scan_topic:=/filtered_scan
```

## デバッグのヒント

- 入力確認: `ros2 topic echo /scan`
- 出力確認: `ros2 topic echo /filtered_scan`
- 落とす点を増やしたい場合: `threshold` を小さくする
- 落とす点を減らしたい場合: `threshold` を 1.0 に近づける
- 別の LiDAR トピックに適用したい場合: `scan_topic` と `filtered_scan_topic` を変更する
