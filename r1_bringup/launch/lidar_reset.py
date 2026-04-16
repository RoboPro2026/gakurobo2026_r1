"""
LiDAR 起動前リセット

urg_open() 内の clear_urg_communication_buffer() は URG が連続スキャン中だと
データが届き続けるためループが抜けられず、無限ブロックになる。
urg_node2 起動前に QT コマンド（計測停止）を送信してデバイスをアイドル状態に戻す。
"""

import os
import time


def reset_lidar(port: str, baud: int = 115200) -> None:
    if not os.path.exists(port):
        print(f"[reset_lidar] {port}: device not found, skipping")
        return
    try:
        import serial  # type: ignore
    except ImportError:
        print("[reset_lidar] WARNING: pyserial not installed, skipping reset")
        return
    try:
        s = serial.Serial(port, baud, timeout=1.0, write_timeout=1.0)
        s.write(b"QT\n")
        time.sleep(0.3)
        s.reset_input_buffer()
        s.close()
        print(f"[reset_lidar] {port}: QT sent successfully")
    except Exception as e:
        print(f"[reset_lidar] {port}: {e} (continuing anyway)")
