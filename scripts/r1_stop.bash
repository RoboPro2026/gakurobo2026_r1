#!/bin/bash
pkill -SIGINT -f "ros2 launch r1_bringup" || true
