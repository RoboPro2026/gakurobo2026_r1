# AGENTS.md

このファイルは Claude Code・Codex などの AI エージェントが作業前に読むためのガイドです。  
コーディング規約の詳細は [`CONTRIBUTING.md`](./CONTRIBUTING.md)、プロジェクト全体の概要は [`README.md`](./README.md) を参照してください。

## 前提環境

- OS: Ubuntu 22.04
- ROS 2: Humble
- 言語: C++20 / Python 3.10
- ワークスペース: `~/ros2_ws`

## ビルド

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

特定パッケージのみビルドする場合:

```bash
colcon build --symlink-install --packages-select r1_main r1_control r1_machine r1_bringup r1_msgs r1_util
```

## 実行してはいけないコマンド

以下は実機ハードウェアを操作するコマンドです。AI エージェントは実行しないでください。

```bash
# 実機起動スクリプト（CAN 初期化・USB 権限付与・CPU 設定を行う）
./src/gakurobo2026_r1/scripts/r1_setup.bash
./src/gakurobo2026_r1/scripts/r1_manual.bash
./src/gakurobo2026_r1/scripts/r1_auto.bash

# Bag 記録（実機トピックを記録する）
./src/gakurobo2026_r1/scripts/record.bash
```

`sudo` を含むコマンドも基本的に実行しないでください。

## パッケージ構成と変更先の判断

| パッケージ | 役割 | 変更例 |
|---|---|---|
| `r1_bringup` | launch・パラメータ管理 | 起動引数の追加、YAML 修正 |
| `r1_main` | 高レベル制御・状態遷移 | 操縦ロジック、モード遷移 |
| `r1_control` | 移動制御・経路追従 | 速度制御、経路生成 |
| `r1_machine` | ハードウェア接続 | モータ・エンコーダ・GPIO |
| `r1_msgs` | ROS メッセージ定義 | 新メッセージ型の追加 |
| `r1_util` | 共通ライブラリ | 座標変換など汎用関数 |

R1 固有の変更はこのリポジトリで行います。  
`sabacan`、`bno086`、`ros2_socketcan` など複数プロジェクトで共有するものは `gakurobo2026_common` 側にあります。このリポジトリからは変更しないでください。

## サブモジュールの扱い

`urg_node2` はサブモジュールです。サブモジュール内のファイルは直接変更しないでください。

## コーディングルール（要約）

詳細は `CONTRIBUTING.md` を参照してください。ここでは AI が特に注意すべき点を挙げます。

### C++

- インデント: スペース 2 個
- 型名: `PascalCase`、関数・変数名: `snake_case`、定数: `UPPER_SNAKE_CASE`
- `private` メンバ変数は末尾 `_`（例: `gain_`）
- publisher: `xxx_publisher_`、subscription: `xxx_subscription_`
- `using namespace std;` は使わない
- 整数型は `int`、浮動小数点型は `double` を基本とする
- 配列は `std::vector` を優先（`std::array` や C 形式配列は原則使わない）
- ループカウンタは `int`。`std::vector` のサイズ比較は `(int)v.size()` にキャストする
- 範囲 `for` より通常の `for` を使う
- 関数・引数に `const` は付けない（ライブラリの都合で必要な場合を除く）
- ラムダ式はコールバック生成以外で使わない
- 名前空間は原則使わない
- コメントは日本語でよい。関数・クラス・構造体には Doxygen 形式のコメントを付ける

### Python

- インデント: スペース 4 個
- 変数・関数名: `snake_case`、定数: `UPPER_SNAKE_CASE`
- PEP 8 に大きく外れない範囲で書く
- 複雑な内包表記や多段の三項演算子は避ける

### ROS 2

- package 名・node 名は先頭を `r1_` で始める
- node 名の末尾は `_node` で終える
- ROS 2 node の実装は特別な理由がない限り C++ を使う
- launch は `yaml + python` を使う
- ログレベル: デバッグ出力は `LOG_INFO`、警告は `LOG_WARN`、エラーは `LOG_ERROR` / `LOG_FATAL`（`LOG_DEBUG` は使わない）
- タイマ周期は必要以上に高くしない
- 不要な topic・node は増やさない

## ドキュメント更新のルール

次に該当する変更をしたときは、関連する Markdown も合わせて更新してください。

- launch の起動方法を変えた
- ROS 2 parameter を追加・削除・改名した
- topic 名や message の意味を変えた
- ノード間の責務や接続関係を変えた
- セットアップ・実行手順を変えた

## PR・コミットのルール

- コミットメッセージは日本語の件名にする（例: `r1_control: 軌道追従目標の選択を修正`）
- PR には変更目的・変更箇所・挙動の変化・再現手順・確認結果を含める
- マージは通常の merge を使う（squash merge は行わない）
- `main` ブランチは常に動く状態を保つ
