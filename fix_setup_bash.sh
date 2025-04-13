#!/bin/bash

# Đường dẫn tới file setup.bash
SETUP_BASH_PATH="/home/huy/ros2_ws/install/setup.bash"

# Thay thế dòng COLCON_CURRENT_PREFIX cũ bằng giá trị mới
sed -i 's|/home/huy/mnt/usb_ext4/opt/ros/rolling|/opt/ros/rolling|' "$SETUP_BASH_PATH"

echo "COLCON_CURRENT_PREFIX đã được cập nhật thành /opt/ros/rolling trong $SETUP_BASH_PATH"
