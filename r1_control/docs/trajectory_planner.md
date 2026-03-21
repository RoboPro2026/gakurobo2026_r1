# r1_control / trajectory_planner ドキュメント

## 概要

`r1_control` パッケージは、移動ロボットの軌道生成を行うためのライブラリ・テストノード・Python バインディング・GUI ツールを提供します。

- C++ ライブラリ `TrajectoryPlanner`（`include/r1_control/trajectory_planner.h`）
- C++ テストノード（`example/test_trajectory_planner_node.cpp` など）
- Python バインディング `trajectory_planner`（`src/trajectory_planner.cpp`）
- 軌道生成スクリプト `run_trajectory_planner.py`
- 軌道生成 GUI `trajectory_planner_gui.py`

本ドキュメントでは数式の詳細には踏み込まず、「何ができるか」「どう使うか」に絞って説明します。

---

## C++: TrajectoryPlanner クラス

### 役割

`TrajectoryPlanner` は以下をまとめて行うクラスです。

- waypoint 列（`x, y`）から 2D スプラインを構成
- 各セグメントの距離を計算
- 最大速度・加速度・躍度・角速度の制約下で台形加速（`AccelDesigner`）による速度プロファイル生成
- 5 次最小躍度軌道（`MinimumJerk`）による姿勢（`theta`）軌道生成
- サンプリング周期 `dt` ごとの状態列（`t, x, y, theta, distance, v_trans, a_trans, j_trans, omega, curvature`）を計算

`theta` と `v_trans` は `x_wp`, `y_wp` と同じ長さの配列として与えます。  
拘束条件を置かない waypoint には `inf` などの**非有限値**を入れ、`TrajectoryPlanner` 側では `std::isfinite()` で有限値だけを拘束条件として使用します。

- `theta_wp[i]` が有限値なら、その waypoint の姿勢拘束として使う
- `v_trans_wp[i]` が有限値なら、その waypoint の並進速度拘束として使う
- `theta_wp[i]` / `v_trans_wp[i]` が非有限値なら、その waypoint では拘束しない

`theta_wp` と `v_trans_wp` は独立に設定できるため、ある waypoint では姿勢だけ拘束し、別の waypoint では速度だけ拘束する、といった使い方もできます。

加えて、**元の waypoint 行で隣り合う**有限値の `v_trans` が同じ値なら、その区間は `AccelDesigner` による加減速区間ではなく、指定速度の**定速区間**として扱います。たとえば `0.3, 0.3, 0.3` と連続して書けば、その連続区間は 0.3 m/s で移動します。空欄（非有限値）が入ると定速区間はそこで切れ、空欄をまたいで同じ値でも定速区間にはなりません。

### 主な API（概要）

- ヘッダ: `include/r1_control/trajectory_planner.h`
- 名前空間なしのクラス `TrajectoryPlanner`
- 代表的なメソッド:
  - `std::vector<int> calc(...)`  
    `x_wp, y_wp, theta_wp, v_trans_wp`（いずれも `std::vector<double>`、ただし `theta_wp` / `v_trans_wp` の未使用要素は非有限値）を与えて軌道を計算し、各区間のステータス配列を返す。
  - `get_trajectory()`  
    Python バインディング内部で使用。計算済みの軌道を複数の `std::vector<double>` のタプルとして返す。
  - `print_csv_trajectory(FILE *fp)`  
    サンプリングされた軌道を CSV 形式で出力。
  - `print_csv_waypoint(FILE *fp)`  
    waypoint を CSV 形式で出力。

### ステータスコード

`calc()` の返り値（各区間のステータス）は `trajectory_planner.h` で以下の意味に定義されています。

- `0`: 正常終了
- `-1`: warning（終端速度が目標速度になっていない）
- `-2`: warning（最大角速度 `omega_max` を超えている）
- `-3`: 失敗

### C++ テストノードの利用方法

`example/test_trajectory_planner_node.cpp` にはいくつかのテストパターンが実装されています。

ビルド済みのワークスペースで、例えば以下のように実行できます。

```bash
ros2 run r1_control test_trajectory_planner_node
```

実行すると、

- `test_trajectory_planner_output.csv`（軌道のサンプル列）
- `test_trajectory_planner_waypoint_output.csv`（使用した waypoint）

がカレントディレクトリに生成されます。  
これらは `example/test_trajectory_planner_plot.py` で可視化できます。

```bash
python3 src/gakurobo2026_r1/r1_control/example/test_trajectory_planner_plot.py
```

---

## Python: trajectory_planner モジュール

### 概要

`src/trajectory_planner.cpp` で pybind11 を用いて C++ クラスをラップし、Python から次の関数を提供しています。

- `trajectory_planner.calculate_trajectory(x_wp, y_wp, theta_wp, v_trans_wp, dt, v_max, a_max, j_max, omega_max)`

引数の型（Python 側のイメージ）:

- `x_wp: list[float]`（長さ >= 3）
- `y_wp: list[float]`（`x_wp` と同じ長さ）
- `theta_wp: list[float]`（`x_wp` と同じ長さ、未使用点は `math.inf` などの非有限値）
- `v_trans_wp: list[float]`（`x_wp` と同じ長さ、未使用点は `math.inf` などの非有限値）

`theta_wp[i]` は waypoint `i` の姿勢拘束値、`v_trans_wp[i]` は waypoint `i` の並進速度拘束値です。  
拘束しない要素には `math.inf`、`float("inf")`、`numpy.inf` などの非有限値を入れてください。  
`theta_wp` と `v_trans_wp` は独立で、同じ waypoint で両方拘束してもよいですし、片方だけ拘束しても構いません。

戻り値は 11 要素のタプルです。

1. `status`（各区間のステータス配列）
2. `t`
3. `x`
4. `y`
5. `theta`
6. `distance`
7. `v_trans`
8. `a_trans`
9. `j_trans`
10. `omega`
11. `curvature`

### 直接利用例（簡略）

```python
import trajectory_planner

status, t, x, y, theta, distance, v_trans, a_trans, j_trans, omega, curvature = (
    trajectory_planner.calculate_trajectory(
        x_wp, y_wp, theta_wp, v_trans_wp,
        dt, v_max, a_max, j_max, omega_max,
    )
)
```

各配列は `list[float]`（`status` だけ `list[int]`）として Python 側に渡されます。

---

## Python: run_trajectory_planner.py

### 概要

`run_trajectory_planner.py` は、Python から軌道生成ライブラリを使いやすくするためのラッパークラス `RunTrajectoryPlanner` を提供しています。

主な機能:

- waypoint CSV の読み書き
- パラメータ CSV の読み書き
- C++ バインディングを使った軌道生成
- 2D 軌道プロット / 時系列プロット

### waypoint / パラメータファイル

`RunTrajectoryPlanner` は「入力ファイルベース」と「出力ファイルベース」という 2 つのプレフィックスからファイル名を決定します。

- 入力 waypoint: `<input_prefix>_waypoints.csv`
- 入力パラメータ: `<input_prefix>_robot_parameter.csv`
- 出力 waypoint: `<output_prefix>_waypoints.csv`
- 出力パラメータ: `<output_prefix>_robot_parameter.csv`

デフォルトでは `output_prefix = input_prefix` として扱われます。

保存時には、同じ内容を日付付きバックアップファイルにも書き出します。

- 例: `trajectory_waypoints.csv` → `backup_trajectory_waypoints_20251212_153045.csv`

### 主なメソッド（抜粋）

- `run(reload_waypoints: bool = True) -> bool`  
  waypoint 読み込み（必要に応じて）→ ゾーン変換 → 軌道計算を行い、失敗があれば `False` を返します。
- `plot()`  
  フィールド＋軌道の 2D 描画と時系列グラフをまとめて表示します。
- `load_waypoint() / save_waypoint()`  
  waypoint CSV の読み書き。
- `load_parameters() / save_parameters()`  
  パラメータ CSV の読み書き。
- `plot_vel_acc_jerk()`  
  速度・加速度・躍度の 3 段サブプロットを表示。
- `plot_curvature()`  
  曲率のグラフを表示。

### ゾーン（blue/red）の扱い

`zone` を `"blue"` / `"red"` で切り替えることで、**blue のときに** Y 軸対称（x の符号反転）した座標系で軌道計算を行います。

内部では:

- 元の waypoint は常に **red ゾーン基準**の座標系で保持（CSV もこの座標系で保存）
- 軌道計算・描画の直前に `zone` に応じて `x` と `theta` を変換（元データは変更しない）

という方針で実装されているため、ゾーンを切り替えても waypoint の数値自体は壊れません。

変換の式（`run_trajectory_planner.py`）:

- `zone == "red"`: 変換なし
- `zone == "blue"`: `x' = -x`, `theta' = π - theta`（`y` はそのまま）

補足:

- フィールド図形は red ゾーン座標で定義されており、`zone == "blue"` のときに X 方向へミラーして描画します。
- 表示上のロボットの向きは、ゾーンを切り替えても同じ見た目になるように（描画時のみ）`theta` を補正して表示します。

### 入力の注意（失敗しやすいポイント）

`TrajectoryPlanner.calc()` 側で入力チェックが入っているため、CSV/GUI で編集するときは次を満たす必要があります。

- waypoint 数（`x,y`）は 3 点以上
- `theta_wp` と `v_trans_wp` は `x_wp`, `y_wp` と同じ長さであること
- `theta_wp[0]` と `theta_wp[-1]` は有限値であること
- `v_trans_wp[0]` と `v_trans_wp[-1]` は有限値であること
- 始点/終点以外は、必要な waypoint にだけ有限値を入れ、未使用点は `inf` にすること

---

## GUI: trajectory_planner_gui.py

### 概要

`trajectory_planner_gui.py` は、`RunTrajectoryPlanner` を使った簡易 GUI です。  
PyQt6 + matplotlib を使い、以下の操作をマウスとテーブル編集で行えます。

- waypoint の可視化・追加・削除・並べ替え
- ゾーン（blue/red）の切り替え（パラメータテーブルから）
- 軌道生成と結果の可視化
- 速度・加速度・躍度・曲率・概要グラフの表示
- 各種 CSV の読み込み・保存
- ログ出力（GUI + コマンドライン）

### 起動方法（例）

ワークスペースルート（`ros2_ws`）から:

```bash
python3 src/gakurobo2026_r1/r1_control/src/trajectory_planner_gui.py
```

※ 仮想環境（venv）を利用している場合は、有効化した上で実行してください。

### 画面構成と主な操作

上から順に:

1. **設定ボタン**
   - `設定読み込み`  
     入力プレフィックスで指定された CSV（`*_waypoints.csv`, `*_robot_parameter.csv`）を読み込み、テーブルとグラフに反映します。
   - `軌道生成`  
     waypoint テーブル・パラメータテーブルの内容を `RunTrajectoryPlanner` に反映し、軌道生成 → 2D グラフ描画を行います。  
     失敗した場合はログにエラーが出力され、距離・時間表示は `N/A` にリセットされます。
   - `設定書き込み`  
     GUI 上の waypoint / パラメータを、それぞれ出力プレフィックスで指定された CSV に保存します（バックアップ付き）。

2. **距離・時間表示 & グラフボタン**
   - `距離: ...`, `時間: ...`  
     直近に生成した軌道の総距離・総時間を表示します。
   - `速度/加速度/躍度`  
     速度・加速度・躍度の 3 段グラフを表示します。
   - `曲率`  
     曲率のみのグラフを表示します。
   - `概要グラフ`  
     位置・姿勢・距離・速度・角速度を 6 段の概要グラフとして表示します。
   - `アニメ再生`  
     生成済みの軌道に沿ってロボット姿勢をアニメーション表示します（再生中は `アニメ停止` に変わります）。
   - `t: ... x: ... y: ... theta: ... v: ...`  
     アニメ再生中の現在フレームの値を表示します（再生していないときは `N/A`）。

3. **ズーム・移動スライダー**
   - `ズーム`  
     フィールド表示を中心を保ったまま拡大縮小します。
   - `X移動`, `Y移動`  
     ズームした状態で表示領域を X/Y 方向にパンします。

4. **フィールド & 軌道表示領域**
   - フィールド上に waypoint（赤い `×`）と軌道、間引きされたロボット姿勢矩形を表示します。
   - マウスクリック:  
     グラフ上をクリックすると、その位置に waypoint を反映します。
     - waypoint テーブルで行を選んだ状態でクリックすると、その行の waypoint を **上書き**します（追加しません）。
     - 何も選択していない場合は末尾に **追加**します。
     - `zone == "blue"` の表示中でも、内部的には red ゾーン基準の座標で保持されるように逆変換して保存します。

5. **右ペイン上部: ファイルベース & パラメータテーブル**
   - `入力ファイルベース` / `出力ファイルベース`  
     それぞれのテキストボックスに base 名（例: `/tmp/trajectory`）を入力すると、
     - 入力: `/tmp/trajectory_waypoints.csv`, `/tmp/trajectory_robot_parameter.csv`
     - 出力: `/tmp/trajectory_waypoints.csv`, `/tmp/trajectory_robot_parameter.csv`
     のようにファイル名が決まります。
   - パラメータテーブル  
     - `zone`（"blue" / "red"）  
     - `dt`, `v_max`, `a_max`, `j_max`, `omega_max`, `robot_dt`  
     を編集すると、「軌道生成」でその値が反映されます。  
     ただし `zone` は編集した時点でプロットにも即時反映されます（表示範囲はリセットされます）。  
     パラメータファイルが存在しない場合はテーブル自体が非表示のままです。

6. **右ペイン下部: waypoint 編集テーブル & ボタン**
   - waypoint テーブル  
     - 列: `x`, `y`, `theta`, `v_trans`（`Index` 列は内部用で非表示）  
     - セルを直接編集して値を変更できます。
   - 編集ボタン
     - `選択を削除`  
       選択行を削除し、後続のインデックスを詰めます。
     - `一つ上へ` / `一つ下へ`  
       選択行を一つ上/下に移動し、`theta` / `v_trans` のインデックスも整合性を保ちながら入れ替えます。
   - 行選択  
     選択中の waypoint はグラフ上で色を変えてハイライト表示されます。

7. **最下部: ログビュー**
   - 軌道生成のステータスやエラーメッセージが、コマンドラインと同様の内容で表示されます。
   - `RunTrajectoryPlanner` が `_log()` 経由で出力するすべてのメッセージがここに流れます。

---

## まとめ

`r1_control` は、

- C++ クラス `TrajectoryPlanner`
- その Python バインディング
- 軌道生成専用 GUI / スクリプト

を通じて、ロボットの軌道計画を一通り試せる環境を提供します。  

- C++ から直接使いたい場合: `TrajectoryPlanner` とテストノード。
- 結果だけ確認したい場合: テストノード + `test_trajectory_planner_plot.py`。
- 直感的に waypoint やパラメータを触りたい場合: `trajectory_planner_gui.py`。

用途に応じてこれらを組み合わせて利用してください。
