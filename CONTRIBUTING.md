# Contributing to gakurobo2026_r1

このドキュメントは、`gakurobo2026_r1` に変更を入れるときの最小限の共通ルールをまとめたものです。  
実機を扱うリポジトリなので、コードの正しさだけでなく、安全性、再現性、検証手順の明確さも重視します。

このプロジェクトでは、主に `C++20` と `Python 3.10` を使用します。  
Python は基本的に `venv` 環境を使用します。  
開発環境は `Ubuntu 22.04`、ROS 2 のバージョンは `Humble` です。  
新しくコードを書くときも、基本的にはこの前提に合わせてください。

## 対象

このリポジトリには主に次の内容が含まれます。

- `r1_bringup`
- `r1_main`
- `r1_control`
- `r1_machine`
- `r1_msgs`
- `r1_util`
- `data`
- `urg_node2` submodule

R1 固有の変更は、まずこのリポジトリで管理してください。  
`sabacan`、`bno086`、`ros2_socketcan` など複数プロジェクトで共有する内容は、必要に応じて `gakurobo2026_common` 側で扱うことを検討してください。

## 基本方針

- 実装はシンプルに保つ
- 変更は小さく分け、目的が分かる単位で commit する
- 挙動が変わる変更では、README や docs も一緒に更新する

ここでいう「シンプル」とは、短いコードであることよりも、読んだ人が責務と挙動を追いやすいことを意味します。  
将来使うかもしれない抽象化、過剰な一般化、見通しを悪くする賢すぎる実装は避けてください。

## 作業前の確認

- 依存関係や起動方法はまず [`README.md`](./README.md) を確認する
- 既存の package 構成と責務に合わせて修正箇所を決める
- 変更対象が submodule かどうかを確認する
- 実機に影響する変更では、影響する topic、parameter、launch 構成を先に洗い出す

## 推奨フロー

1. 目的と影響範囲を整理する
2. 関連する node、launch、parameter、topic を確認する
3. 実装する
4. `colcon build` と必要な手動確認を行う
5. 関連ドキュメントを更新する
6. PR に再現手順と確認結果を書く

## ブランチ運用

- `main` ブランチは、常に動く状態を保つ
- 機能追加や修正を行うときは、`main` から別ブランチを切って開発する
- ブランチ名はなんでもいい
- Pull Request をマージするときは、通常の merge を使う
- 使い終わったブランチは必ず削除する
- `squash merge` などでコミット履歴を打ち消す行為は、トラブルの元になるため行わない
- ただし、やむを得ない `force push` は許容するが、注意して行うこと

## C++ のコーディング規約

このリポジトリの C++ は、ROS 2 node とその周辺ライブラリが中心です。  
見た目の統一だけでなく、実機でのデバッグしやすさも重視してください。

### 命名規則や基本設定のルール

- 既存ファイルの書き方に合わせる
- インデントはスペース 2 個を基本とする
- 波かっこは既存コードと同様に改行スタイルを優先する
- 型名は `PascalCase`、関数名と変数名は `snake_case` を基本とする
- 定数名は `UPPER_SNAKE_CASE` を基本とする
- `#include` は、標準ライブラリを `<>`、それ以外を `""` で書く
- `private` メンバ変数は末尾 `_` を付ける。
- `public` メンバ変数に末尾 `_` を付けるかどうかは任意とする
- 定数は意味が明確なら `constexpr` を優先する
- `auto` は型が明白なときだけ使う。読みにくくなる箇所では避ける
- publisher 名は原則 `parameter_name_publisher_` とする
- subscription 名は原則 `parameter_name_subscription_` とする

### コメントと整形

- 日本語コメントでよい
- コメントにはなぜその実装にしたかを書く。
- 関数、クラス、構造体にはdoxygen形式のコメントをつける。
- プログラムが何をしているかも、プログラムを斜め読みしてわかるくらいには書いておく。
- ソースコード中に `TODO` や `NOTE` を残してもよい
- C++ のコードを編集したときは、自分たちで書いたコードに限って `clang-format` を適用してよい
  - 既存ファイル全体を無差別に整形して、関係ない差分を広げない


### このリポジトリにおける実装ルール

個人の好みが強く反映されているため、このリポジトリでのみのルールとします。

- `using namespace std;`は使わない。
- 値の範囲や有効桁数に問題がなければ、整数型は `int`、浮動小数点型は `double` を使う
  - 整数型が`long long`ではなく、`int`なのは長年慣れ親しんでいるからという理由です。
- 配列的なデータは `std::vector` を優先して使う
  - `C` 形式の配列や `std::array` は原則使わない
  - ただし、将来的にマイコンへ移植する可能性があるプログラムでは、例外として `C` 形式の配列を認める
- `std::vector` の `push_back()` は、要素数が多いときは事前に `reserve()` するか、`resize()` で領域確保してから使う
- 個人的に通常の `for` 文のほうが読みやすいため、範囲 `for` ではなく通常の `for` 文を使う
- `for`文のカウンタ変数は `int` 型で統一する。`std::vector`に対して使うときは`(int)a.size()`みたいにする。
- 参照とポインタのどちらでもよい場面では、参照ではなくポインタを使う
- クラスを受け渡すときなどの所有表現は、 `std::shared_ptr` で統一する
  - `std::unique_ptr` による厳密な所有権分離は、行わなくてよい
- 参照を使ってよいのは、既存の変数、とくに既存のメンバ変数へ別名を付けて短く扱いたい場面を主とする
  - 例: `int & step = mode1_detect_origin_step_;`
- `C` 形式の cast と `static_cast` のどちらでもよい場面では、`C` 形式の cast を使ってよい
  - これは `static_cast` が長く、読みにくいため
- デバッグ出力や標準出力、ファイル出力では、`std::cout` や `std::stringstream`、`std::ofstream` より `printf`、`fprintf`、`FILE *` などの `C` 系 API を優先してよい
  - これは `std::cout` だと出力フォーマットを自由に調整しにくく、使いにくいため
- 関数の引数や関数自体に`const`はつけない。
  - 人間が使うときに書き換えなければいいだけなので。
  - ただし、ライブラリの引数が`const`になっているときは仕方がないのでつける。
- `private`のメンバ変数にアクセスするときはsetterやgetter関数を使うこと。
  - 原則として、メンバ変数は`private`とする。
  - `private`にするとsetterやgetter関数が多くなるとき、メンバ変数を`public`にしたほうが実装がシンプルになるとき（バッファに直接アクセスなど）は`public`にしてよい。
- 高度な template meta プログラミングは行わない
  - 可読性が悪く、メンテナンスもしにくいため。templateを使う場合は、シンプルな実装に留めること。
- 自分たちで書くプログラムは、なるべく継承を使わないで実装する。（rclcpp::Nodeを除く）
  - 継承を使うと、プログラムが複雑になるため。
  - 継承を使うのはハードウェア（ドライバ関連）が強いが、このリポジトリではハードウェアを扱うことは少ないのも理由の一つ。
- ラムダ式は関数callback作成以外で使わないこと。
  - 関数の中に関数（のようなもの）があると違和感があるので、ヘルパー関数にラムダ式は使わないこと。
- 名前空間は使わない。
  - この規模感なら名前空間がなくてもなんとかなるので。
  - ただし、CANのIDなど、多数のパラメータがあって命名規則が冗長になるときは名前空間を使う。
- 文字列とenumの変換を行う際は、必要であれば`magic_enum`を使って良い。

通常の `for` 文は次のような形を基本とします。

```cpp
for (int i = 0; i < (int)values.size(); i++) {
  // ...
}
```

ROS 2 node の例:

```cpp
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class R1ExampleNode : public rclcpp::Node
{
public:
  R1ExampleNode() : Node("r1_example_node")
  {
    this->declare_parameter<double>("gain", 1.0);
    this->get_parameter("gain", gain_);

    example_value_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/example_value", 10);

    gain_ref_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/gain_ref", 10,
      std::bind(&R1ExampleNode::gain_ref_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&R1ExampleNode::timer_callback, this));
  }

private:
  void gain_ref_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    gain_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "gain=%f", gain_);
  }

  void timer_callback()
  {
    std::vector<double> values = {1.0, 2.0, 3.0};
    double sum = 0.0;

    for (int i = 0; i < (int)values.size(); i++) {
      sum += values[i];
    }

    std_msgs::msg::Float64 msg;
    msg.data = gain_ * sum;
    example_value_publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "count=%d sum=%f value=%f", count_, sum, msg.data);
    count_++;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr example_value_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gain_ref_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  double gain_ = 1.0;
  int count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<R1ExampleNode>());
  rclcpp::shutdown();
  return 0;
}
```

## Python のコーディング規約

このリポジトリの Python は、launch ファイル、GUI、補助スクリプトが中心です。  
短いスクリプトでも、後から読み返して修正しやすい形を優先してください。

- PEP 8 を大きく外さない
- インデントはスペース 4 個を使う
- 変数名と関数名は `snake_case` を基本とする
- 定数は `UPPER_SNAKE_CASE` を使う
- 1 文字変数は、短いループ変数など必要最小限にとどめる
- 長い処理は小さな関数に分ける
- `main` 相当の処理があるスクリプトは、可能なら関数化する
- launch ファイルでも、ノード生成や条件分岐が増えたら補助関数へ分ける
- 例外を無言で握りつぶさない
- ログや `print()` は、何を確認するための出力か分かる形にする
- リストや配列の走査は、添字が必要ないなら素直な書き方でよい
- ただし、複雑な内包表記や多段の三項演算子は避ける
- 現在の Python フォーマットは、VSCode の拡張機能に入っている formatter を使用する
- ただし、この運用は再現性が弱いため、将来的には VSCode に依存しない共通のフォーマット仕組みを導入する予定

Python では C++ と違い、無理に `for i in range(...)` へ統一する必要はありません。  
ただし、添字が必要な処理では、意図が分かる明示的なループを使ってください。

追加の方針:

- import は、標準ライブラリ、サードパーティライブラリ、自作モジュールの順にまとめる
- CSV や設定ファイルを読む処理は、読み込み部分と処理部分を関数で分ける
- データを複数列扱うときは、列名や意味が分かる変数名を使う
- 例外が起きうる入出力処理では、最低限どこで失敗したか分かるようにする
- 補助スクリプトでも、入力ファイルの前提をコードかコメントで分かるようにする

TODO: いい感じの Python の例を追加する

## ROS 2 の規約

このリポジトリは ROS 2 package 群として運用しているため、ROS 2 固有の命名や parameter 設計も揃えます。

### 基本ルール

- package 名や node 名は、基本的に先頭を `r1` で始める
- node 名の末尾は `node` で終える
  - この node 名ルールは、運用開始当初に決めた命名ルールを維持するためのもので、深い意味はない
- ROS 2 の node 実装は、ライブラリや GUI の都合など特別な理由がない限り、Python ではなく C++ を使う
- launch は `yaml + python` を使う
- デバッグ用のログは`LOG_INFO`、Waringは`LOG_WARN`、エラーや致命的なエラーは`LOG_ERROR`もしくは`LOG_FATAL`にする。`LOG_DEBUG`はROS 2自体のログがいろいろ出てきて見にくいので使わない。
- ROS 2が重いので、少しでもPCの負荷が少なくなるように次のことを心がける。
- 通常運用時のログレベルは `warn` 以上とし、少しでもログが少なくなるようにする。標準出力は重いので。
- ROS 2 のタイマ周期は、必要以上に高くしない
- 不要な topic は、意図的なデバッグ用 topic を除いてなるべく減らす
- 不要な node は起動しない


### Parameter と YAML

- ROS 2 parameter にするかどうかは、変更頻度で判断する
- 変更頻度の高い値は parameter 化する
- 変更頻度の低い値は、実装をシンプルに保つためにソースコード中の変数や定数として持つ
- parameter は、`for` 文や条件分岐で組み立てる必要がある場合を除いて、基本的に YAML に書く
- YAML ファイルには、なるべくその parameter が何を意味するか分かるコメントを付ける
- 特に調整値、単位がある値、既定値の意図が分かりにくい値にはコメントを残す
- parameter を増やしすぎて運用を複雑にしない

YAML の例:

```yaml
r1_example_node:
  ros__parameters:
    # 制御タイマ周期 [Hz]
    timer_rate: 100.0

    # publish 先 topic 名
    output_topic: "/example_value"

    # 出力に掛けるゲイン
    gain: 1.0
```

launch の例:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    example_node = Node(
        package="r1_example",
        executable="r1_example_node",
        name="r1_example_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=["config/r1_example_config.yaml"],
    )

    return LaunchDescription([example_node])
```

## CMake の規約

このリポジトリの CMake は、`ament_cmake` を使った ROS 2 package のビルド定義です。  
ビルドが通るだけでなく、依存関係と生成物の関係が読みやすいことを重視してください。

- build type は原則 `Release` に統一する
  - 理由は早くするため。`-O3`は`Release`ビルドの場合はつけてもつけなくても変わらないらしいのでそのままにしている
- CMake コマンドは既存ファイルに合わせて小文字を基本とする
- package ごとの `CMakeLists.txt` は、構成をなるべく揃える
- パフォーマンスや依存関係、可読性に問題がなければ、項目はアルファベット順に揃える
- `find_package()` は必要なものだけを書く
- 依存関係は不足より過剰を避ける
- `ament_target_dependencies()` と `target_link_libraries()` の役割を分けて書く
- `target_include_directories()`、`install()`、`ament_package()` の流れを崩しすぎない
- 同じ include path を複数 target で使う場合は、既存のように共通変数へまとめてよい
- warning オプション、`CMAKE_CXX_STANDARD`、`ccache`、build type の扱いは package 間でむやみにばらつかせない
- 新しい executable や module を追加したら、依存関係、include path、install まで忘れずに書く

CMake で避けたいこと:

- 使っていない `find_package()` を残すこと
- 実際には不要なライブラリを全 target に雑に link すること
- package ごとに標準バージョンや warning 方針を不必要に分裂させること
- ローカル環境前提のパスや設定を直書きすること

新しい package や target を追加するときは、既存の [`r1_control/CMakeLists.txt`](./r1_control/CMakeLists.txt) や [`r1_machine/CMakeLists.txt`](./r1_machine/CMakeLists.txt) の構成を基準にしてください。

例:

```cmake
cmake_minimum_required(VERSION 3.8)
project(r1_example)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(r1_example_node src/r1_example_node.cpp)

ament_target_dependencies(r1_example_node
  rclcpp
  std_msgs
)

install(TARGETS
  r1_example_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

## ビルドと確認

変更内容に応じて、少なくとも次のどれかは実施してください。

- リポジトリ全体のビルド

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

- 変更した package のみビルド

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select r1_main r1_control r1_machine r1_bringup r1_msgs r1_util
```

- シミュレーション起動確認

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch r1_bringup r1_bringup.launch.py use_sim:=true
```

- 必要に応じた node 単体確認

```bash
ros2 run r1_control r1_chassis_control_node --ros-args \
  --params-file src/gakurobo2026_r1/r1_bringup/config/r1_machine_config.yaml
```

実機確認をした場合は、次を記録してください。

- 使用した launch 引数
- 使用した機体構成
- 確認した topic や動作
- 想定どおりだった点と未確認の点

自動テストが十分でない変更では、PR に手動確認手順を必ず書いてください。

## ドキュメント更新

次に該当する変更では、関連する Markdown の更新を基本とします。

- launch の起動方法を変えた
- parameter を追加、削除、改名した
- topic 名や message の意味を変えた
- ノード間の責務や接続関係を変えた
- 実行手順やセットアップ手順を変えた

主な参照先:

- [`README.md`](./README.md)
- [`r1_bringup/README.md`](./r1_bringup/README.md)
- [`r1_control/README.md`](./r1_control/README.md)
- [`r1_machine/README.md`](./r1_machine/README.md)
- [`r1_main_node.md`](./r1_main/docs/r1_main_node.md)

## Pull Request に書くこと

PR には少なくとも次を含めてください。

- 変更の目的
- 変更した package / node / launch
- ユーザーや操縦者から見た挙動の変化。互換性のない変更があった場合は明記してください。

挙動変更があるのに確認結果がない PR は、レビューしにくく事故要因になります。

## コミットメッセージ

厳密な形式は必須ではありませんが、少なくとも内容が一目で分かる日本語の件名にしてください。

例:

- `r1_control: 軌道追従目標の選択を修正`
- `r1_bringup: use_sim と use_lidar の挙動をREADMEに追記`
- `r1_machine: mecanum のパラメータ処理を更新`

ただしSSHでミニPCにアクセスしている関係上、デバッグのために雑なコミットメッセージでcommitすることは認めます。

例:

- `update`
- `update parameter`
