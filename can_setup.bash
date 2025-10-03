#!/bin/bash
#
# CANインターフェースを安定的・継続的に運用するための設定スクリプト

# --- ユーザー設定項目 ---
DEFAULT_INTERFACE="can0"
BITRATE="1000000"
SAMPLE_POINT="0.875" 
RESTART_MS="100"
TX_QUEUE_LEN="10000"
# ------------------------

set -e

if [ "$EUID" -ne 0 ]; then
  echo "エラー: このスクリプトはroot権限(sudo)で実行する必要があります。"
  exit 1
fi

INTERFACE=${1:-$DEFAULT_INTERFACE}

echo "--- 安定化設定を開始します: ${INTERFACE} ---"

echo "[1/4] インターフェースを停止しています..."
ip link set ${INTERFACE} down

echo "[2/4] ビットレート、サンプルポイント、自動復帰を設定中..."
ip link set ${INTERFACE} type can \
    bitrate ${BITRATE} \
    sample-point ${SAMPLE_POINT} \
    restart-ms ${RESTART_MS} 

echo "[3/4] 送信キューの長さを設定中: ${TX_QUEUE_LEN}"
ifconfig ${INTERFACE} txqueuelen ${TX_QUEUE_LEN}

echo "[4/4] インターフェースを起動しています..."
ifconfig ${INTERFACE} up

echo "--- 設定が完了しました ---"
echo "現在の ${INTERFACE} の状態:"
ip -details link show ${INTERFACE}