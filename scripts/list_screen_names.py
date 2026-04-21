#!/usr/bin/env python3

import os
import sys

from PyQt6.QtWidgets import QApplication


def main() -> int:
    # Wayland セッションでは Qt が xcb を選ぶと libxcb-cursor0 不足で落ちるため、
    # 明示指定がない場合だけ Wayland backend を優先する。
    if "QT_QPA_PLATFORM" not in os.environ and "WAYLAND_DISPLAY" in os.environ:
        os.environ["QT_QPA_PLATFORM"] = "wayland"

    app = QApplication.instance() or QApplication(sys.argv[:1])
    screens = app.screens()
    primary_screen = app.primaryScreen()

    if len(screens) == 0:
        print("No screen names found.", file=sys.stderr)
        print("Try running this script from a logged-in desktop session.", file=sys.stderr)
        return 1

    for index, screen in enumerate(screens):
        geometry = screen.geometry()
        primary_mark = " primary" if screen == primary_screen else ""
        print(
            f"{index}: name={screen.name()}{primary_mark} "
            f"geometry={geometry.width()}x{geometry.height()}+{geometry.x()}+{geometry.y()}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
