#!/usr/bin/env bash

echo "========== R1 bag record start =========="

cd "$HOME/ros2_ws"
source .venv/bin/activate
source install/setup.bash

# CAN ブリッジ系と Sabacan 系の topic は bag から除外する。
# TODO: これでもrecordが重い場合はdebugと名のつくものと、scan1とscan2を除外するようにする
CAN_TOPIC_REGEX='(^|/)(sabacan_[^/]*|(from|to)_can_bus[^/]*)$'

echo "exclude topic regex: ${CAN_TOPIC_REGEX}"

ros2 bag record -a --exclude "${CAN_TOPIC_REGEX}" "$@"
