#!/usr/bin/env python3

import sys

from PyQt6.QtWidgets import QApplication


def main() -> int:
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
