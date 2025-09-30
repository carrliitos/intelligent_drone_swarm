#!/bin/bash
echo "=== $(date) ===" >> camera_settings_log.txt
v4l2-ctl -d /dev/video2 --all >> camera_settings_log.txt
