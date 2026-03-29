echo "========== R1 setup start =========="

cd "$HOME/ros2_ws"
source install/setup.bash

sudo cpupower frequency-set -g performance

IMU_DEV="/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30FJEIM-if00-port0"
LIDAR_DEV="/dev/ttyACM0"

# by-id -> 実体の ttyUSBx を取得
IMU_TTY="$(basename "$(readlink -f "$IMU_DEV")")"

sudo chmod 666 "$IMU_DEV"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/$IMU_TTY/latency_timer"

sudo chmod 666 "$LIDAR_DEV"

echo "---------- IMU setup OK ----------"

sudo ./src/gakurobo2026_common/can_setup.bash

echo "--------- USB2CAN setup OK ----------"
echo "========= R1 setup success =========="
