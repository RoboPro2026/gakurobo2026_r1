#!/bin/bash
set -e

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DESKTOP_DIR="$(xdg-user-dir DESKTOP)"

ICON_DIR="$HOME/.local/share/icons/hicolor/scalable/apps"
mkdir -p "$ICON_DIR"
ln -sf "$REPO_DIR/icons/r1_auto_blue.svg" "$ICON_DIR/r1_auto_blue.svg"
ln -sf "$REPO_DIR/icons/r1_auto_red.svg" "$ICON_DIR/r1_auto_red.svg"
ln -sf "$REPO_DIR/icons/r1_stop.svg" "$ICON_DIR/r1_stop.svg"
ln -sf "$REPO_DIR/icons/r1_record.svg" "$ICON_DIR/r1_record.svg"
gtk-update-icon-cache -f -t "$HOME/.local/share/icons/hicolor" 2>/dev/null || true

chmod +x "$REPO_DIR/scripts/r1_auto.bash"
chmod +x "$REPO_DIR/scripts/r1_stop.bash"
chmod +x "$REPO_DIR/scripts/record.bash"
chmod +x "$REPO_DIR/desktop/r1_auto_blue.desktop"
chmod +x "$REPO_DIR/desktop/r1_auto_red.desktop"
chmod +x "$REPO_DIR/desktop/r1_stop.desktop"

ln -sf "$REPO_DIR/desktop/r1_auto_blue.desktop" "$DESKTOP_DIR/r1_auto_blue.desktop"
ln -sf "$REPO_DIR/desktop/r1_auto_red.desktop" "$DESKTOP_DIR/r1_auto_red.desktop"
ln -sf "$REPO_DIR/desktop/r1_stop.desktop" "$DESKTOP_DIR/r1_stop.desktop"

gio set "$DESKTOP_DIR/r1_auto_blue.desktop" metadata::trusted true
gio set "$DESKTOP_DIR/r1_auto_red.desktop" metadata::trusted true
gio set "$DESKTOP_DIR/r1_stop.desktop" metadata::trusted true

echo "デスクトップアイコンを設定しました: $DESKTOP_DIR"
